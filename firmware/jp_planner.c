#include "jp_planner.h"
#include <limits.h>
#include <string.h>

static int calibration_valid(const jp_calibration *c) {
    return c && c->speed_mm_s >= 50 && c->speed_mm_s <= 5000 &&
        c->vehicle_width_mm >= 1 && c->vehicle_width_mm <= 10000 &&
        c->clearance_mm <= 5000 && c->left_ms <= 60000 && c->right_ms <= 60000 &&
        c->uturn_ms <= 60000 && c->settle_ms <= 60000 && c->risk_penalty_ms <= 60000;
}

static jp_step make_step(const jp_edge *e, uint16_t index, uint8_t incoming,
                         const jp_calibration *c) {
    jp_step step;
    memset(&step, 0, sizeof(step));
    step.edge_index = index;
    step.src = e->src; step.dst = e->dst; step.heading = e->heading;
    step.action = (uint8_t)((e->heading + 4 - incoming) % 4);
    if (step.action == JP_LEFT) step.turn_ms = c->left_ms;
    if (step.action == JP_RIGHT) step.turn_ms = c->right_ms;
    if (step.action == JP_UTURN) step.turn_ms = c->uturn_ms;
    if (step.action != JP_STRAIGHT) step.turn_ms += c->settle_ms;
    step.travel_ms = ((uint32_t)e->length_mm * 1000 + c->speed_mm_s - 1) / c->speed_mm_s;
    step.risk_cost_ms = ((uint32_t)e->risk_permille * c->risk_penalty_ms + 999) / 1000;
    step.length_mm = e->length_mm; step.risk_permille = e->risk_permille;
    return step;
}

static uint32_t step_cost(const jp_step *s, uint8_t policy) {
    if (policy == JP_DISTANCE) return s->length_mm;
    return s->travel_ms + s->turn_ms + (policy == JP_ROBUST ? s->risk_cost_ms : 0);
}

int jp_plan(const jp_graph *g, uint8_t start, uint8_t goal, uint8_t heading,
            uint8_t policy, const jp_calibration *c,
            jp_workspace *w, jp_route *out) {
    uint16_t states, initial, i, iter;
    int16_t end = -1;
    if (out) memset(out, 0, sizeof(*out));
    if (!g || !w || !out || !calibration_valid(c) || g->node_count < 1 ||
        g->node_count > JP_MAX_NODES || g->edge_count > JP_MAX_EDGES ||
        start >= g->node_count || goal >= g->node_count || heading > 3 || policy > JP_ROBUST)
        return JP_INVALID;
    for (i = 0; i < g->edge_count; ++i) {
        const jp_edge *e = &g->edges[i];
        uint16_t j;
        if (e->src >= g->node_count || e->dst >= g->node_count || e->src == e->dst ||
            e->heading > 3 || e->length_mm < 1 || e->length_mm > 10000 ||
            e->width_mm < 1 || e->width_mm > 10000 || e->risk_permille > 1000 || e->blocked > 1)
            return JP_INVALID;
        for (j = 0; j < i; ++j)
            if (g->edges[j].src == e->src && g->edges[j].dst == e->dst) return JP_INVALID;
    }
    states = (uint16_t)(g->node_count * 4);
    initial = (uint16_t)(start * 4 + heading);
    for (i = 0; i < states; ++i) {
        w->distance[i] = UINT32_MAX;
        w->parent[i] = -1; w->parent_edge[i] = -1; w->settled[i] = 0;
    }
    w->distance[initial] = 0;
    for (iter = 0; iter < states; ++iter) {
        int16_t best = -1;
        uint32_t best_cost = UINT32_MAX;
        for (i = 0; i < states; ++i)
            if (!w->settled[i] && w->distance[i] < best_cost) {
                best = (int16_t)i; best_cost = w->distance[i];
            }
        if (best < 0) break;
        if (best / 4 == goal) { end = best; break; }
        w->settled[best] = 1;
        for (i = 0; i < g->edge_count; ++i) {
            const jp_edge *e = &g->edges[i];
            uint16_t next;
            jp_step step;
            uint32_t candidate;
            if (e->src != best / 4 || e->blocked ||
                e->width_mm < (uint32_t)c->vehicle_width_mm + 2U * c->clearance_mm) continue;
            step = make_step(e, i, (uint8_t)(best % 4), c);
            next = (uint16_t)(e->dst * 4 + e->heading);
            /* Accepted parameter bounds keep total cost below UINT32_MAX. */
            candidate = best_cost + step_cost(&step, policy);
            if (candidate < w->distance[next]) {
                w->distance[next] = candidate; w->parent[next] = best;
                w->parent_edge[next] = (int16_t)i;
            }
        }
    }
    if (end < 0) return JP_NO_ROUTE;
    out->cost = w->distance[end]; out->revision = g->revision;
    while (end != (int16_t)initial) {
        int16_t prev = w->parent[end], edge_index = w->parent_edge[end];
        jp_step step;
        if (prev < 0 || edge_index < 0 || out->step_count >= JP_MAX_STEPS) return JP_INVALID;
        step = make_step(&g->edges[edge_index], (uint16_t)edge_index, (uint8_t)(prev % 4), c);
        out->steps[out->step_count++] = step;
        out->nominal_time_ms += step.travel_ms + step.turn_ms;
        out->distance_mm += step.length_mm;
        if (step.action != JP_STRAIGHT) ++out->turn_count;
        end = prev;
    }
    for (i = 0; i < out->step_count / 2; ++i) {
        jp_step tmp = out->steps[i];
        out->steps[i] = out->steps[out->step_count - 1 - i];
        out->steps[out->step_count - 1 - i] = tmp;
    }
    return JP_OK;
}

size_t jp_workspace_size(void) { return sizeof(jp_workspace); }
size_t jp_graph_size(void) { return sizeof(jp_graph); }
size_t jp_route_size(void) { return sizeof(jp_route); }
