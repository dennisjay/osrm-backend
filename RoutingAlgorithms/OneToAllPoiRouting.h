//
//  OneToAllPoiRouting.h
//  osrm-backend
//
//  Created by Dennis Jöst on 01.11.14.
//  Copyright (c) 2014 Dennis Jöst. All rights reserved.
//

#ifndef osrm_backend_OneToAllPoiRouting_h
#define osrm_backend_OneToAllPoiRouting_h

#include "ManyToManyRouting.h"
#include "../Util/TimingUtil.h"



template <class DataFacadeT> class OneToAllPoiRouting final : public BasicRoutingInterface<DataFacadeT>
{
    using super = BasicRoutingInterface<DataFacadeT>;
    std::vector<PhantomNode> poi_vector ;
    
    using QueryHeap = SearchEngineData::QueryHeap;
    SearchEngineData &engine_working_data;
    
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;

    ManyToManyRouting<DataFacadeT> many ;
    SearchSpaceWithBuckets search_space_with_buckets ;
    
public:
    OneToAllPoiRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
    : super(facade), engine_working_data(engine_working_data), many(facade, engine_working_data)
    {
        poi_vector = facade->GetPoisPhantomNodeList() ;
        SimpleLogger().Write() << "loaded pois to Routing class";
        
        engine_working_data.InitializeOrClearFirstThreadLocalStorage(super::facade->GetNumberOfNodes());
        QueryHeap &query_heap = *(engine_working_data.forwardHeap);
        
        
        SimpleLogger().Write() << "Performing poi backward searches";
        TIMER_START(poi_backward_searches);
        unsigned target_id = 0;
        
        for (const PhantomNode &phantom_node: poi_vector)
        {
            query_heap.Clear();
            // insert target(s) at distance 0
            
            if (SPECIAL_NODEID != phantom_node.forward_node_id)
            {
                query_heap.Insert(phantom_node.forward_node_id,
                                    phantom_node.GetForwardWeightPlusOffset(),
                                    phantom_node.forward_node_id);
            }
            if (SPECIAL_NODEID != phantom_node.reverse_node_id)
            {
                query_heap.Insert(phantom_node.reverse_node_id,
                                    phantom_node.GetReverseWeightPlusOffset(),
                                    phantom_node.reverse_node_id);
            }
            
            // explore search space
            while (!query_heap.Empty())
            {
                many.BackwardRoutingStep(target_id, query_heap, search_space_with_buckets);
            }
            ++target_id;
        }
        TIMER_STOP(poi_backward_searches);
        SimpleLogger().Write() << "ok, after " << TIMER_SEC(poi_backward_searches) << "s" << std::endl;
        
        
    }
    
    ~OneToAllPoiRouting() {}
    
    std::shared_ptr<std::vector<EdgeWeight>> operator()(const PhantomNode &phantom_node, int distance)
    const
    {
        const unsigned number_of_locations = static_cast<unsigned>(poi_vector.size() + 1);
        std::shared_ptr<std::vector<EdgeWeight>> result_table =
        std::make_shared<std::vector<EdgeWeight>>(number_of_locations,
                                                  std::numeric_limits<EdgeWeight>::max());
        
        engine_working_data.InitializeOrClearFirstThreadLocalStorage(super::facade->GetNumberOfNodes());
        
        QueryHeap &query_heap = *(engine_working_data.forwardHeap);
        
        // for each source do forward search (only one)
        const unsigned source_id = 0;
        query_heap.Insert(phantom_node.reverse_node_id,
                          -phantom_node.GetReverseWeightPlusOffset(),
                          phantom_node.reverse_node_id);
        
        // explore search space
        while (!query_heap.Empty())
        {
            many.ForwardRoutingStep(source_id,
                                number_of_locations,
                                query_heap,
                                search_space_with_buckets,
                                result_table);
        }
            

        return result_table;
        
    }
    
    
};


#endif
