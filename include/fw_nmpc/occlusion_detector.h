#ifndef OCCLUSION_DETECTOR_H_
#define OCCLUSION_DETECTOR_H_

namespace fw_nmpc {

enum IndexOcclusionAttributes {
    IDX_OCC_POS = 0, // occlusion detection position
    IDX_OCC_NORMAL = 3, // occlusion normal
    IDX_OCC_RAY_LEN = 7 // ray length (only in ray list XXX: this may be a bad way to do this)
}; // occlusion attributes



/*
    OcclusionDetector class
    tools for detecting occlusions in a uniform elevation map and storing them in a circular buffer
 */
class OcclusionDetector {

    public:

        OcclusionDetector();

        // constants
        static constexpr int LEN_DATA_MAX = 101;       // maximum number of detections to store in one buffer rung
        static constexpr int LEN_BUFFER_MAX = 10;      // maximum length of horizon buffer (how many previous (in time) collections of occlusion detections are stored)
        static constexpr int NUM_OCC_ATTRIBUTES = 6;   // number of occlusion attributes (position[3],normal[3])

        // TODO: is there a better way (without tons of copies) than exposing these arrays publicly?

        // ray casting list
        double ray_list_[LEN_DATA_MAX][NUM_OCC_ATTRIBUTES+1]; // ENU -- user's responsibility to externally fill this..

        // storage arrays
        int detections_[LEN_BUFFER_MAX][LEN_DATA_MAX];                          // nodes at which we have detected an occlusion
        double attributes_[LEN_BUFFER_MAX][LEN_DATA_MAX][NUM_OCC_ATTRIBUTES];   // occlusion attributes

        // sets
        void setBufferLength(int len);
        void setDataLength(int len);

        // gets
        int getBufferHead() { return buffer_head_; }
        int getDetectionCountAtHead() { return detection_count_[buffer_head_]; }

        // functions
        void castRays(const double map_origin_north, const double map_origin_east, const int map_height,
            const int map_width, const double map_resolution, const double *map);
        // XXX: MAYBE we can keep the map relevant operations totally in this class eventually?? (or in grid map...)
        void clear();
        // void clearDataTail(int len);
        void rotate();

    private:

        // constants
        static constexpr double EPSILON = 0.000001;    // used for triangle checks

        // arrays
        int detection_count_[LEN_BUFFER_MAX];

        // variables
        int buffer_head_;
        int len_buffer_;
        int len_data_;

        // functions
        int castRay(double *occ_position, double *occ_normal, double *occ_vertex_1, double *occ_vertex_2, double *occ_vertex_3,
            double *ray_origin, const double ray_length, double *ray_direction,
            const double map_origin_north, const double map_origin_east, const int map_height,
            const int map_width, const double map_resolution, const double *map);
        int intersectTriangle(double *p_occ, double *n_occ, const double r0[3], const double v_ray[3],
            const double p1[3], const double p2[3], const double p3[3], const int spin);
}; // class OcclusionDetector

} // namespace fw_nmpc

#endif // OCCLUSION_DETECTOR_H_
