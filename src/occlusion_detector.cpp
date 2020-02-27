#include <fw_nmpc/occlusion_detector.h>
#include <fw_nmpc/common/helpers.h>

namespace fw_nmpc {

OcclusionDetector::OcclusionDetector() :
    buffer_head_(0),
    len_buffer_(1),
    len_data_(1)
{
    clear();
} // OcclusionDetector

void OcclusionDetector::setBufferLength(int len)
{
    if (len > LEN_BUFFER_MAX) {
        len_buffer_ = LEN_BUFFER_MAX;
    }
    else if (len < 1) {
        len_buffer_ = 1;
    }
    else {
        len_buffer_ = len;
    }
    clear();
} // setBufferLength

// void OcclusionDetector::setDataLength(int len)
// {
//     int temp_len;
//     if (len > LEN_DATA_MAX) {
//         temp_len = LEN_DATA_MAX;
//     }
//     else if (len < 1) {
//         temp_len = 1;
//     }
//     else {
//         temp_len = len;
//     }
//
//     if (temp_len < len_data_) clearDataTail(temp_len);
//     len_data_ = temp_len;
// } // setDataLength

void OcclusionDetector::setDataLength(int len)
{
    if (len > LEN_DATA_MAX) {
        len_data_ = LEN_DATA_MAX;
    }
    else if (len < 1) {
        len_data_ = 1;
    }
    else {
        len_data_ = len;
    }
    clear();
} // setDataLength

void OcclusionDetector::castRays(const double map_origin_north, const double map_origin_east, const int map_height,
    const int map_width, const double map_resolution, const double *map)
{
    // initialize/allocate
    double occ_position[3], occ_normal[3], occ_vertex_1[3], occ_vertex_2[3], occ_vertex_3[3], ray_length;

    // zero detection count
    detection_count_[buffer_head_] = 0;

    for (int i = 0; i < len_data_; i++) {
        // calculate ray length
        ray_length = ray_list_[i][IDX_OCC_RAY_LEN];

        // re-init occ pos
        occ_position[0] = 0.0;
        occ_position[1] = 0.0;
        occ_position[2] = 0.0;

        // cast the ray
        detections_[buffer_head_][i] = castRay(occ_position, occ_normal, occ_vertex_1, occ_vertex_2, occ_vertex_3,
            &ray_list_[i][IDX_OCC_POS], ray_length, &ray_list_[i][IDX_OCC_NORMAL],
            map_origin_north, map_origin_east, map_height, map_width, map_resolution, map);

        // count detections
        detection_count_[buffer_head_] += (detections_[buffer_head_][i] > 0);

        // update attributes
        // ENU -> NED
        attributes_[buffer_head_][i][IDX_OCC_POS] = occ_position[1];
        attributes_[buffer_head_][i][IDX_OCC_POS+1] = occ_position[0];
        attributes_[buffer_head_][i][IDX_OCC_POS+2] = -occ_position[2];
        attributes_[buffer_head_][i][IDX_OCC_NORMAL] = occ_normal[1];
        attributes_[buffer_head_][i][IDX_OCC_NORMAL+1] = occ_normal[0];
        attributes_[buffer_head_][i][IDX_OCC_NORMAL+2] = -occ_normal[2];
    }
} // castRays

void OcclusionDetector::clear()
{
    // reset buffer head
    buffer_head_ = 0;

    // zero all nodes
    for (int i = 0; i < LEN_BUFFER_MAX; i++) {
        detection_count_[i] = 0;
        for (int j = 0; j < LEN_DATA_MAX; j++) {
            detections_[i][j] = 0;
            for (int k = 0; k < NUM_OCC_ATTRIBUTES; k++) {
                attributes_[i][j][k] = 0.0;
            }
        }
    }
    for (int j = 0; j < LEN_DATA_MAX; j++) {
        for (int k = 0; k < NUM_OCC_ATTRIBUTES+1; k++) {
            ray_list_[j][k] = 0.0;
        }
    }
} // clear

// void OcclusionDetector::clearDataTail(int len)
// {
//     // zero all newly unused nodes
//     for (int i = 0; i < LEN_BUFFER_MAX; i++) {
//         for (int j = len; j < len_data_; i++) {
//             detections_[i][j] = 0;
//             for (int k = 0; k < NUM_OCC_ATTRIBUTES; j++) {
//                 attributes_[i][j][k] = 0.0;
//             }
//         }
//     }
// } // clearDatatail

void OcclusionDetector::rotate()
{
    if (++buffer_head_ >= len_buffer_) buffer_head_ = 0; // increment / wrap
} // rotateHead

/* cast ray through terrain map and determine the intersection point on any occluding trianglular surface */
int OcclusionDetector::castRay(double *occ_position, double *occ_normal, double *occ_vertex_1, double *occ_vertex_2, double *occ_vertex_3,
    double *ray_origin, const double ray_length, double *ray_direction,
    const double map_origin_north, const double map_origin_east, const int map_height,
    const int map_width, const double map_resolution, const double *map) {

    /*
        INPUTS:
        (double) ray_origin[3]              ray origin (e,n,u) [m]
        (double) ray_length                 ray length [m]
        (double) ray_direction[3]           ray direction - unit vector (e,n,u)
        (double) map_origin_north           northing origin of terrain map [m]
        (double) map_origin_east            easting origin of terrain map [m]
        (int) map_height                    map grid height
        (int) map_width                     map grid width
        (double) map_resolution             terrain discretization [m]
        (double) map[map_height*map_width]  terrain map [m]

        OUTPUTS:
        (double) occ_position[3]    coord. of the ray-triangle intersection [m]
        (double) occ_normal[3]      unit normal vector to triangle
        (double) occ_vertex_1[3]    coord. of triangle vertex 1 (e,n,u) [m]
        (double) occ_vertex_2[3]    coord. of triangle vertex 2 (e,n,u) [m]
        (double) occ_vertex_3[3]    coord. of triangle vertex 3 (e,n,u) [m]

        RETURNS:
        (int) occ_detected   0 = no detection; 1 = BR (bottom-right) triangle detected; 2 = TL (top-left) triangle detected
     */

    const int map_height_1 = map_height - 1;
    const int map_width_1 = map_width - 1;

    /* initialize */
    int occ_detected = 0;

    /* relative (unit) start position */
    const double x0 = (ray_origin[0] - map_origin_east)/map_resolution;
    const double y0 = (ray_origin[1] - map_origin_north)/map_resolution;

    /* initial height */
    const double h0 = ray_origin[2];

    /* vector for triangle intersect inputs */
    const double r0_rel[3] = {x0*map_resolution,y0*map_resolution,h0};

    /* relative end position */
    const double x1 = (ray_origin[0] + ray_length * ray_direction[0] - map_origin_east)/map_resolution;
    const double y1 = (ray_origin[1] + ray_length * ray_direction[1] - map_origin_north)/map_resolution;

    /* end height */
    const double h1 = ray_origin[2] + ray_length * ray_direction[2];

    /* line deltas */
    const double dx = fabs(x1 - x0);
    const double dy = fabs(y1 - y0);

    /* initial cell origin */
    int x = (int)(floor(x0));
    int y = (int)(floor(y0));

    /* unit change in line length per x/y (see definition below) */
    double dt_dx;
    double dt_dy;

    /* change in height per unit line length (t) */
    const double dh = h1 - h0;

    /* number of cells we pass through */
    int n = fabs(floor(x1)-x) + fabs(floor(y1)-y) + 1; /*XXX: what is the real difference between this and using dx / dy? */

    /* initialize stepping criteria */
    double t_next_horizontal, t_last_horizontal;
    double t_next_vertical, t_last_vertical;
    double x_inc, y_inc;

    if (dx < 0.00001) {
        x_inc = 0.0;
        dt_dx = INFINITY;
        t_next_horizontal = INFINITY;
        t_last_horizontal = INFINITY;
    }
    else if (x1 > x0) {
        x_inc = 1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x + 1.0 - x0) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x0 - x) * dt_dx;
    }
    else {
        x_inc = -1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x0 - x) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x + 1.0 - x0) * dt_dx;
    }

    if (dy < 0.00001) {
        y_inc = 0.0;
        dt_dy = INFINITY;
        t_next_vertical = INFINITY;
        t_last_vertical = INFINITY;
    }
    else if (y1 > y0) {
        y_inc = 1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y + 1.0 - y0) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y0 - y) * dt_dy;
    }
    else {
        y_inc = -1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y0 - y) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y + 1.0 - y0) * dt_dy;
    }

    /* find cell intersection in opposite direction to initialize cell entrance
     * condition */
    bool last_step_was_vert = (t_last_vertical < t_last_horizontal);

    /* initialize entrance height */
    double h_entr = h0;

    /* for loop init */
    int ret;
    double t, h_exit, h_check;
    bool take_vert_step, check1, check2, check3, check4;

    /* check that start position is not already under the terrain */

    /* bound corner coordinates */
    int x_check = constrain(x, 0, map_width_1);
    int y_check = constrain(y, 0, map_height_1);
    int x_check1 = constrain(x_check+1, 0, map_width_1);
    int y_check1 = constrain(y_check+1, 0, map_height_1);
    /* convert to row-major indices */
    int idx_corner1 = y_check*map_width + x_check;
    int idx_corner2 = y_check1*map_width + x_check;
    int idx_corner3 = y_check1*map_width + x_check1;
    int idx_corner4 = y_check*map_width + x_check1;

    const double x0_unit = x0 - x;
    const double y0_unit = y0 - y;
    if (y0_unit > x0_unit) {
        /* check top-left triangle */
        const double ht = map[idx_corner1] + x0_unit*(map[idx_corner3]-map[idx_corner2]) + y0_unit*(map[idx_corner2] - map[idx_corner1]);
        if (ht > h0) {
            return occ_detected;
        }
    }
    else {
        /* check bottom-right triangle */
        const double ht = map[idx_corner1] + x0_unit*(map[idx_corner4]-map[idx_corner1]) + y0_unit*(map[idx_corner3] - map[idx_corner4]);
        if (ht > h0) {
            return occ_detected;
        }
    }

    /* cast the ray */
    int i;
    for (i = 0; i < n; i=i+1) {

        /* check the next step we will take and compute the exit height */
        if (t_next_vertical < t_next_horizontal) {
            /* next step is vertical */
            take_vert_step = true;
            t = t_next_vertical; /* current step */
            t_next_vertical = t_next_vertical + dt_dy;
        }
        else {
            /* next step is horizontal */
            take_vert_step = false;
            t = t_next_horizontal; /* current step */
            t_next_horizontal = t_next_horizontal + dt_dx;
        }

        /* take minimum of entrance and exit height for check */
        /* TODO: should be a way to get rid of this if statement by looking at dh outside for loop... */
        h_exit = h0 + dh * t;
        if (dh > 0.0) {
            h_check = h_entr;
        }
        else {
            h_check = h_exit;
        }
        h_entr = h_exit;

        /* bound corner coordinates */
        x_check = constrain(x, 0, map_width_1);
        y_check = constrain(y, 0, map_height_1);
        x_check1 = constrain(x_check+1, 0, map_width_1);
        y_check1 = constrain(y_check+1, 0, map_height_1);
        /* convert to row-major indices */
        idx_corner1 = y_check*map_width + x_check;
        idx_corner2 = y_check1*map_width + x_check;
        idx_corner3 = y_check1*map_width + x_check1;
        idx_corner4 = y_check*map_width + x_check1;
        /* check the four corners */
        check1 = map[idx_corner1] > h_check; /* corner 1 (bottom left) */
        check2 = map[idx_corner2] > h_check; /* corner 2 (top left) */
        check3 = map[idx_corner3] > h_check; /* corner 3 (top right) */
        check4 = map[idx_corner4] > h_check; /* corner 4 (bottom right) */

        /* check cell triangles */
        if (last_step_was_vert) { /* / / / / / / / / / / / / / / / / / / */
            /* vertical entrance step */

            if (take_vert_step) {
                /* next step is vertical */

                if (y_inc > 0) { /*TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop... */
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && occ_detected==0) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
            }
            else  {/* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (y_inc > 0 && x_inc > 0) {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else if (y_inc < 0 && x_inc < 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        }
        else { /* last step was horizontal / / / / / / / / / / / / / / / */
            if (take_vert_step) {
                /* next step is vertical */

                if (x_inc > 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    if ((y_inc < 0) && (occ_detected==0)) {
                        /* BR */

                        /* check bottom-right triangle corners */
                        if (check1 || check4 || check3) {

                            /* set 3 corners */
                            occ_vertex_1[0] = map_resolution*x;
                            occ_vertex_1[1] = map_resolution*y;
                            occ_vertex_1[2] = map[idx_corner1];
                            occ_vertex_2[0] = map_resolution*(x+1);
                            occ_vertex_2[1] = map_resolution*y;
                            occ_vertex_2[2] = map[idx_corner4];
                            occ_vertex_3[0] = map_resolution*(x+1);
                            occ_vertex_3[1] = map_resolution*(y+1);
                            occ_vertex_3[2] = map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                            occ_detected += ret; /* =1 if detection */
                        }
                    }
                }
                else {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    if ((y > 0) && (occ_detected==0)) {
                        /* TL */

                        /* check top-left triangle corners */
                        if (check1 || check2 || check3) {

                            /* set 3 corners */
                            occ_vertex_1[0] = map_resolution*x;
                            occ_vertex_1[1] = map_resolution*y;
                            occ_vertex_1[2] = map[idx_corner1];
                            occ_vertex_2[0] = map_resolution*x;
                            occ_vertex_2[1] = map_resolution*(y+1);
                            occ_vertex_2[2] = map[idx_corner2];
                            occ_vertex_3[0] = map_resolution*(x+1);
                            occ_vertex_3[1] = map_resolution*(y+1);
                            occ_vertex_3[2] = map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                            occ_detected += ret*2; /* =2 if detection */
                        }
                    }
                }
            }
            else { /* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (x_inc > 0) {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*(x+1);
                        occ_vertex_2[1] = map_resolution*y;
                        occ_vertex_2[2] = map[idx_corner4];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        occ_vertex_1[0] = map_resolution*x;
                        occ_vertex_1[1] = map_resolution*y;
                        occ_vertex_1[2] = map[idx_corner1];
                        occ_vertex_2[0] = map_resolution*x;
                        occ_vertex_2[1] = map_resolution*(y+1);
                        occ_vertex_2[2] = map[idx_corner2];
                        occ_vertex_3[0] = map_resolution*(x+1);
                        occ_vertex_3[1] = map_resolution*(y+1);
                        occ_vertex_3[2] = map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersectTriangle(occ_position, occ_normal, r0_rel, ray_direction, occ_vertex_1, occ_vertex_2, occ_vertex_3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        } /* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

        /* return if occlusion detected */
        if (occ_detected) {
            // shift back origin -- TODO: make it so we dont subtract and add this..
            occ_position[0] += map_origin_east;
            occ_position[1] += map_origin_north;

            return occ_detected;
        }

        /* actually take the step */
        if (take_vert_step) { /* (t_next_vertical < t_next_horizontal) */
            /* take a vertical step */
            y = y + y_inc;
        }
        else {
            /* take a horizontal step */
            x = x + x_inc;
        }
        last_step_was_vert = take_vert_step;
    }

    return occ_detected;
} // castRay

/* check ray-triangle intersection */
int OcclusionDetector::intersectTriangle(double *p_occ, double *n_occ,
    const double r0[3], const double v_ray[3], const double p1[3], const double p2[3], const double p3[3], const int spin) {
    /*
        Following: "Fast, Minimum Storage Ray/Triangle Intersection",
        Moeller et. al., Journal of Graphics Tools, Vol.2(1), 1997
    */

    // NOTE: all vectors in here are E,N,U

    // find vectors for two edges sharing p1
    const double e1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    const double e2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};

    // begin calculating determinant - also used to calculate U parameter
    double pvec[3];
    cross(pvec, v_ray, e2);

    /*  XXX:
        We don't test for backwards culling here as, assuming this ray casting
        algorithm is properly detecting the first occluding triangles, we should
        not run into a case of a true "backwards" facing triangle (also the
        current BR and TL definitions have opposite vertex spin definitions..
        should probably change that in the future.. could maybe avoid a few
        divisions of the determinant)
    */

    // if the determinant is near zero, ray lies in the triangle plane
    const double det = dot(e1, pvec);
    if (det > -EPSILON && det < EPSILON) {
        return 0;
    }

    // divide the determinant (XXX: could possibly find a way to avoid this until the last minute possible..)
    const double inv_det = 1.0 / det;

    // calculate distance from p1 to ray origin
    const double tvec[3] = {r0[0] - p1[0], r0[1] - p1[1], r0[2] - p1[2]};

    // calculate u parameter and test bounds
    const double u = dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0) {
        return 0;
    }

    // prepare to test v parameter
    double qvec[3];
    cross(qvec, tvec, e1);

    // calculate v parameter and test bounds
    const double v = dot(v_ray, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) {
        return 0;
    }

    // calculate d_occ, scale parameters, ray intersects triangle
    //*d_occ = dot(e2, qvec) * inv_det;

    // calculate and return intersection point
    const double one_u_v = (1 - u - v);
    p_occ[0] = one_u_v * p1[0] + u * p2[0] + v * p3[0];
    p_occ[1] = one_u_v * p1[1] + u * p2[1] + v * p3[1];
    p_occ[2] = one_u_v * p1[2] + u * p2[2] + v * p3[2];

    // calculate and return plane normal
    cross(n_occ, e2, e1);
    const double one_over_norm_n_occ = 1.0/sqrt(dot(n_occ,n_occ));
    n_occ[0] *= spin * one_over_norm_n_occ;
    n_occ[1] *= spin * one_over_norm_n_occ;
    n_occ[2] *= spin * one_over_norm_n_occ;

    return 1;
} // intersectTriangle

} // namespace fw_nmpc
