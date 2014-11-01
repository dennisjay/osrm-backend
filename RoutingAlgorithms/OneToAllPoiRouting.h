//
//  OneToAllPoiRouting.h
//  osrm-backend
//
//  Created by Dennis Jöst on 01.11.14.
//  Copyright (c) 2014 Dennis Jöst. All rights reserved.
//

#ifndef osrm_backend_OneToAllPoiRouting_h
#define osrm_backend_OneToAllPoiRouting_h

#include "BasicRoutingInterface.h"
#include "../DataStructures/SearchEngineData.h"
#include "../typedefs.h"

#include <boost/assert.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>


template <class DataFacadeT> class OneToAllPoiRouting final : public BasicRoutingInterface<DataFacadeT>
{
    using super = BasicRoutingInterface<DataFacadeT>;
    using QueryHeap = SearchEngineData::QueryHeap;
    SearchEngineData &engine_working_data;
    
    struct NodeBucket
    {
        unsigned target_id; // essentially a row in the distance matrix
        EdgeWeight distance;
        NodeBucket(const unsigned target_id, const EdgeWeight distance)
        : target_id(target_id), distance(distance)
        {
        }
    };
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;
    
public:
    OneToAllPoiRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
    : super(facade), engine_working_data(engine_working_data)
    {
    }
    
    ~OneToAllPoiRouting() {}
    
};


#endif
