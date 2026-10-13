#ifndef JP_PLANNER_H
#define JP_PLANNER_H
#include <stdint.h>
#include <stddef.h>

#define JP_MAX_NODES 64
#define JP_MAX_EDGES 256
#define JP_STATES (JP_MAX_NODES * 4)
#define JP_MAX_STEPS JP_STATES

enum { JP_N=0, JP_E=1, JP_S=2, JP_W=3 };
enum { JP_DISTANCE=0, JP_FASTEST=1, JP_ROBUST=2 };
enum { JP_STRAIGHT=0, JP_RIGHT=1, JP_UTURN=2, JP_LEFT=3 };
enum { JP_OK=0, JP_NO_ROUTE=1, JP_INVALID=2 };

typedef struct {
    uint8_t src, dst, heading, blocked;
    uint16_t length_mm, width_mm, risk_permille;
} jp_edge;

typedef struct {
    uint16_t node_count, edge_count;
    uint32_t revision;
    jp_edge edges[JP_MAX_EDGES];
} jp_graph;

typedef struct {
    uint16_t speed_mm_s, left_ms, right_ms, uturn_ms, settle_ms;
    uint16_t vehicle_width_mm, clearance_mm, risk_penalty_ms;
} jp_calibration;

typedef struct {
    uint16_t edge_index;
    uint8_t src, dst, heading, action;
    uint32_t travel_ms, turn_ms, risk_cost_ms;
    uint16_t length_mm, risk_permille;
} jp_step;

typedef struct {
    uint32_t cost, nominal_time_ms, distance_mm;
    uint16_t turn_count, step_count;
    uint32_t revision;
    jp_step steps[JP_MAX_STEPS];
} jp_route;

typedef struct {
    uint32_t distance[JP_STATES];
    int16_t parent[JP_STATES], parent_edge[JP_STATES];
    uint8_t settled[JP_STATES];
} jp_workspace;

int jp_plan(const jp_graph *graph, uint8_t start, uint8_t goal, uint8_t heading,
            uint8_t policy, const jp_calibration *calibration,
            jp_workspace *workspace, jp_route *output);
size_t jp_workspace_size(void);
size_t jp_graph_size(void);
size_t jp_route_size(void);

#endif
