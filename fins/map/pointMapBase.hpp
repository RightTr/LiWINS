#ifndef POINT_MAP_BASE_HPP
#define POINT_MAP_BASE_HPP

struct BoxPointType
{
    float vertex_min[3];
    float vertex_max[3];
};

template<typename PointType>
class PointMapBase {
public:
    struct MapConfig
    {
        MapConfig(float resolution, std::size_t capacity, 
            float delete_param, float balance_param, float box_length)
            : resolution_(resolution), capacity_(capacity), delete_param_(delete_param), 
                balance_param_(balance_param), box_length_(box_length) {}
        
        MapConfig() = default;
        // OctVoxMap options
        float resolution_ = 0.5;
        std::size_t capacity_ = 1000000;

        // ikd-Tree options
        float delete_param_ = 0.5;
        float balance_param_ = 0.6;
        float box_length_ = 0.1;
        
    };
    
    using Ptr = std::shared_ptr<PointMapBase>;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    virtual ~PointMapBase() = default;

    virtual void InsertPoints(PointVector& Points, bool downsample = false) = 0;

    virtual void Nearest_Search(const PointType& PointQuery, 
                                int k_nearest, 
                                PointVector& Nearest_Points, 
                                std::vector<float>& Point_Distance, 
                                float max_dist = INFINITY) = 0;

    virtual void Build(PointVector input) = 0;

    virtual int Size() = 0;

    virtual void Clear() = 0;

    virtual bool IsInitialized() = 0;

    virtual int Delete_Point_Boxes(std::vector<BoxPointType> &BoxPoints) = 0;

    virtual void GetMap(PointVector& MapPoints) = 0;

    virtual void SetConfig(const MapConfig& config) = 0;
    
    PointVector PCL_Storage;
};

#endif // POINT_MAP_BASE_HPP