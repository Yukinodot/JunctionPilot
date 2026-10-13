#include "jp_planner.h"
#include "jp_guard.h"
#include <stdio.h>

static jp_workspace workspace;
static jp_route route;

int main(void) {
    jp_graph graph = {0};
    jp_calibration c = {300, 850, 650, 1600, 100, 150, 20, 8000};
    jp_guard guard = {0,0,1,0};
    int status;
    graph.node_count = 3; graph.edge_count = 2;
    graph.edges[0] = (jp_edge){0,1,JP_E,0,200,300,0};
    graph.edges[1] = (jp_edge){1,2,JP_N,0,200,300,0};
    status = jp_plan(&graph,0,2,JP_E,JP_FASTEST,&c,&workspace,&route);
    if (status != JP_OK) return 1;
    printf("status=%d cost=%u ms steps=%u turns=%u workspace=%zu bytes\n",
           status,(unsigned)route.cost,(unsigned)route.step_count,
           (unsigned)route.turn_count,jp_workspace_size());
    if (route.cost != 2284 || route.step_count != 2) return 2;
    if (jp_motion_allowed(&guard,1,1,0,1,3) != JP_HOLD) return 3;
    puts("stale plan guard: HOLD");
    return 0;
}
