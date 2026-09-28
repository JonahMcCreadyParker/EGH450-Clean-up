#include <breadcrumb/AStar.h>
#include <algorithm>
#include <limits>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_) const
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Vec2i operator - (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x - right_.x, left_.y - right_.y };
}

uint AStar::Vec2i::distance(const Vec2i& other_) const
{
    auto delta = std::move(*this - other_);
    return sqrt(delta.x * delta.x + delta.y * delta.y);
}

AStar::Node::Node(
    Vec2i coordinates_,
    const Node *parent_
)
{
    parent = parent_;
    coordinates = coordinates_;

    H = 0;

    G = std::numeric_limits<
        decltype(G)
    >::max();

    D = std::numeric_limits<
        decltype(D)
    >::max();
}

AStar::uint AStar::Node::getScore() const
{
    return G + H;
}

AStar::Generator::Generator()
{
    setThetaStar(false);
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

uint AStar::Generator::getStraightLineCost(uint step) const {
    return ((step < 4) ? 10 : 14);
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::setThetaStar(bool enable_)
{
    use_theta_star = enable_;
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    //XXX: http://www.gameaipro.com/GameAIPro2/GameAIPro2_Chapter16_Theta_Star_for_Any-Angle_Pathfinding.pdf

    //open: = closed: = Ø ;
    const Node *current = nullptr;
    NodeSet openSet;
    ConstNodeSet closedSet;
    //g(S_start):= 0;
    //parent(S_start) : = S_start;
    const auto start = new Node(source_);
    start->parent = start;
    start->G = 0;
    start->D = 0;

    //open.Insert(S_start,S_start) + h (S_start));
    openSet.insert(start);

    // ------------------------------------------------------------
    // Request-specific movement policy
    //
    // Axis-aligned requests are the lawnmower-style legs where we
    // want clear rectangular obstacle avoidance.
    //
    // Other requests may use diagonals and Theta* to travel more
    // directly.
    // ------------------------------------------------------------

    const bool axis_aligned_request =
        (
            source_.x == target_.x
            || source_.y == target_.y
        );

    const bool theta_for_request =
        (
            use_theta_star
            && !axis_aligned_request
        );

    //While open != Ø do
    while (!openSet.empty()) {
        //s: = open.Pop();
        current = *openSet.begin();

        // --------------------------------------------------------
        // Equal-cost path tie-break
        //
        // If two A* nodes have the same score, prefer the node
        // closest to the original straight-line route between
        // source and target.
        //
        // This encourages obstacle avoidance to behave as a local
        // detour:
        //
        //     leave nominal path
        //          ↓
        //     avoid obstacle
        //          ↓
        //     rejoin nominal path as soon as safely possible
        //
        // rather than remaining offset until the final waypoint.
        // --------------------------------------------------------

        auto crossTrackDistance =
            [&source_, &target_](const Node* node)
            -> long long {

                const long long route_dx =
                    static_cast<long long>(
                        target_.x
                        - source_.x
                    );

                const long long route_dy =
                    static_cast<long long>(
                        target_.y
                        - source_.y
                    );

                const long long point_dx =
                    static_cast<long long>(
                        node->coordinates.x
                        - source_.x
                    );

                const long long point_dy =
                    static_cast<long long>(
                        node->coordinates.y
                        - source_.y
                    );

                // Magnitude of the 2-D cross product.
                //
                // The actual perpendicular distance would divide
                // this by the route length. Since route length is
                // identical for every node in this search, the
                // division is unnecessary for comparison.
                long long cross =
                    route_dx * point_dy
                    - route_dy * point_dx;

                if (cross < 0) {
                    cross = -cross;
                }

                return cross;
            };

        // current = *openSet.begin();

        for (const auto node : openSet) {

            // Primary comparison:
            // choose the lowest normal A* F-score.

            if (
                node->getScore()
                < current->getScore()
            ) {

                current = node;
            }

            // Secondary comparison:
            // for equally short candidate routes in an
            // axis-aligned search, prefer the route that has
            // accumulated less deviation from the nominal line.

            else if (
                node->getScore()
                == current->getScore()
                && axis_aligned_request
                && node->D < current->D
            ) {

                current = node;
            }

            // Final tie-break:
            // prefer whichever node is geometrically closer
            // to the requested destination.

            else if (
                node->getScore()
                == current->getScore()
                && (
                    !axis_aligned_request
                    || node->D == current->D
                )
                && node->H < current->H
            ) {

                current = node;
            }
        }

        //if s = s_goal then
        //  return “path found”;
        if (current->coordinates == target_) {
            break;
        }

        //closed: = closed U {s};
        closedSet.insert(current);
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));

        // --------------------------------------------------------
        // Movement policy
        //
        // Diagonal movement remains available globally, but when
        // the requested path is already aligned with either the
        // X or Y axis, restrict this individual search to cardinal
        // movement only.
        //
        // This preserves efficient diagonal movement for general
        // navigation while producing clear rectangular detours for
        // the axis-aligned lawnmower search pattern.
        // --------------------------------------------------------

        const uint active_directions =
            axis_aligned_request
                ? std::min<uint>(
                    4,
                    directions
                )
                : directions;

        //foreach s’ E neighbor_vis(s) do
        for (
            uint i = 0;
            i < active_directions;
            ++i
        ) {
            //if s΄ !E closed then
            //  continue
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            //XXX: if s' \E open then
            //XXX: current: s
            //XXX: successor: s'

            //if s΄ !E open then
            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->H = heuristic(successor->coordinates, target_);
                //open.Insert(s΄,g(s’)+ h(s΄));
                openSet.insert(successor);
            }

            //UpdateVertex(s,s΄);

            //gold : = g(s΄);
            //ComputeCost(s,s΄);
            const auto cost_selection =
                computeCost(
                    current,
                    successor,
                    getStraightLineCost(i),
                    theta_for_request
                );

            if (
                cost_selection.parent
            ) {

                // ------------------------------------------------
                // Secondary deviation cost
                //
                // For a horizontal lawnmower leg:
                //     deviation = |y - original_y|
                //
                // For a vertical lawnmower leg:
                //     deviation = |x - original_x|
                //
                // Accumulating this value means two equally short
                // paths are differentiated by how long they spend
                // away from the intended lawnmower line.
                // ------------------------------------------------

                uint candidate_D = 0;

                if (
                    axis_aligned_request
                ) {

                    uint cross_track_cells = 0;

                    if (
                        source_.y
                        == target_.y
                    ) {

                        cross_track_cells =
                            static_cast<uint>(
                                std::abs(
                                    successor->coordinates.y
                                    - source_.y
                                )
                            );
                    }

                    else {

                        cross_track_cells =
                            static_cast<uint>(
                                std::abs(
                                    successor->coordinates.x
                                    - source_.x
                                )
                            );
                    }

                    candidate_D =
                        cost_selection.parent->D
                        + cross_track_cells;
                }

                // ------------------------------------------------
                // Primary optimisation:
                //     lowest real movement cost
                //
                // Secondary optimisation:
                //     lowest accumulated deviation
                // ------------------------------------------------

                if (
                    cost_selection.G
                    < successor->G
                    ||
                    (
                        cost_selection.G
                        == successor->G
                        && axis_aligned_request
                        && candidate_D
                           < successor->D
                    )
                ) {

                    successor->parent =
                        cost_selection.parent;

                    successor->G =
                        cost_selection.G;

                    successor->D =
                        candidate_D;
                }
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        //Create the path from the current (target/null) back to the start (where current is its own parent)
        current = current->parent != current ? current->parent : nullptr;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::NodeCost AStar::Generator::computeCost(
    const Node* current,
    Node* successor,
    const uint c,
    const bool use_theta_for_request
) const {
    //... g(s) + c(s,s΄);
    const uint totalCost = current->G + c;

    if (
        use_theta_for_request
        && lineOfSight(
            current->parent,
            successor
        )
    ) {
        const int dx =
            current->parent->coordinates.x
            - successor->coordinates.x;

        const int dy =
            current->parent->coordinates.y
            - successor->coordinates.y;

        // Keep Theta* line-of-sight cost on the same scale as
        // normal A* movement:
        //   straight grid step = 10
        //   diagonal grid step ≈ 14
        const uint los_cost =
            current->parent->G
            + static_cast<uint>(
                10.0 * sqrt(
                    dx * dx
                    + dy * dy
                )
            );
        if (los_cost < successor->G) {
            return { current->parent, los_cost };
            // successor->parent = current->parent;
            // successor->G = los_cost;
        }
    }
    else if (totalCost < successor->G) {
        // if g(s) + c(s,s΄) < g(s΄) then
        //     parent(s΄):= s;
        //     g(s΄):= g(s) + c(s,s΄);
        return { current, totalCost };
        // successor->parent = current;
        // successor->G = totalCost;
    }

    return {nullptr, std::numeric_limits<decltype(current->G)>::max() };
}

bool AStar::Generator::lineOfSight(const Node* current, const Node* successor) const {
    //https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    int x0 = current->coordinates.x;
    int y0 = current->coordinates.y;
    const int x1 = successor->coordinates.x;
    const int y1 = successor->coordinates.y;

    const int dx = abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while(true) {
        if (detectCollision({x0, y0})) return false;

        if (x0 == x1 && y0 == y1) break;

        const int e2 = 2 * error;

        if(e2 >= dy) {
            if (x0 == x1) break;
            error = error + dy;
            x0 = x0 + sx;
        }

        if(e2 <= dx) {
            if (y0 == y1) break;
            error = error + dx;
            y0 = y0 + sy;
        }
    }

    return true;
}

const AStar::Node* AStar::Generator::findNodeOnList(ConstNodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

void AStar::Generator::releaseNodes(ConstNodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_) const
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * std::sqrt(std::pow(delta.x, 2) + std::pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
