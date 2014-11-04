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
#include "../Util/TimingUtil.h"



template <class DataFacadeT> class OneToAllPoiRouting final : public BasicRoutingInterface<DataFacadeT>
{
    struct NodeBucket
    {
        EdgeWeight distance;
        NodeID target_node;
        NodeBucket(const EdgeWeight distance, const NodeID target_node)
        : distance(distance), target_node(target_node)
        {
        }
    };
    
    using super = BasicRoutingInterface<DataFacadeT>;
    std::vector<PhantomNode> poi_vector ;
    
    using QueryHeap = SearchEngineData::QueryHeap;
    SearchEngineData &engine_working_data;
    
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;
    SearchSpaceWithBuckets search_space_with_buckets ;
    

public:
    OneToAllPoiRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
    : super(facade), engine_working_data(engine_working_data)
    {
       
    }
    
    ~OneToAllPoiRouting() {}
    
    
    void initBackwardRouting() {
        poi_vector = super::facade->GetPoisPhantomNodeList() ;
        SimpleLogger().Write() << "loaded pois to Routing class";
        
        engine_working_data.InitializeOrClearFirstThreadLocalStorage(super::facade->GetNumberOfNodes());
        QueryHeap &query_heap = *(engine_working_data.forwardHeap);
        
        
        SimpleLogger().Write() << "Performing poi backward searches";
        TIMER_START(poi_backward_searches);
        
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
            /*if (SPECIAL_NODEID != phantom_node.reverse_node_id)
            {
                query_heap.Insert(phantom_node.reverse_node_id,
                                  phantom_node.GetReverseWeightPlusOffset(),
                                  phantom_node.reverse_node_id);
            }*/
            
            // explore search space
            while (!query_heap.Empty())
            {
                BackwardRoutingStep(phantom_node.forward_node_id, query_heap, search_space_with_buckets);
            }
        }
        TIMER_STOP(poi_backward_searches);
        SimpleLogger().Write() << "ok, after " << TIMER_SEC(poi_backward_searches) << "s";
        
        SimpleLogger().Write() << "Bucket size: " << search_space_with_buckets.size() ;
    }
    
    std::shared_ptr<std::unordered_map<NodeID, EdgeWeight>> operator()(const PhantomNode &phantom_node, unsigned distance_limit)
    const
    {
        const unsigned number_of_locations = static_cast<unsigned>(poi_vector.size() + 1);
        std::shared_ptr<std::unordered_map<NodeID, EdgeWeight>> result_table =
        std::make_shared<std::unordered_map<NodeID, EdgeWeight>>(number_of_locations);
        
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
            ForwardRoutingStep(distance_limit, source_id,
                                number_of_locations,
                                query_heap,
                                search_space_with_buckets,
                                result_table);
        }
            

        return result_table;
        
    }
    
    
    
    void ForwardRoutingStep(const unsigned distance_limit,
                            const unsigned source_id,
                            const unsigned number_of_locations,
                            QueryHeap &query_heap,
                            const SearchSpaceWithBuckets &search_space_with_buckets,
                            std::shared_ptr<std::unordered_map<NodeID, EdgeWeight>> result_table) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int source_distance = query_heap.GetKey(node);
        
        // check if each encountered node has an entry
        const auto bucket_iterator = search_space_with_buckets.find(node);
        // iterate bucket if there exists one
        if (bucket_iterator != search_space_with_buckets.end())
        {
            const std::vector<NodeBucket> &bucket_list = bucket_iterator->second;
            for (const NodeBucket &current_bucket : bucket_list)
            {
                // get target id from bucket entry
                const int target_distance = current_bucket.distance;
                const NodeID target_node = current_bucket.target_node ;
                const EdgeWeight current_distance =
                (*result_table).find( target_node ) != result_table->end() ? (*result_table)[target_node] : std::numeric_limits<EdgeWeight>::max() ;
                // check if new distance is better
                const EdgeWeight new_distance = source_distance + target_distance;
                if (new_distance >= 0 && new_distance < current_distance)
                {
                    if( (source_distance + target_distance) <= distance_limit ) {
                        (*result_table)[target_node] =
                        (source_distance + target_distance);
                    }
                }
            }
        }
        if (StallAtNode<true>(node, source_distance, query_heap))
        {
            return;
        }
        
        RelaxOutgoingEdges<true>(node, source_distance, query_heap);
    }
    
    void BackwardRoutingStep(NodeID sourceNode, QueryHeap &query_heap,
                             SearchSpaceWithBuckets &search_space_with_buckets) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int target_distance = query_heap.GetKey(node);
        
        // store settled nodes in search space bucket
        search_space_with_buckets[node].emplace_back(target_distance, sourceNode);
        
        SimpleLogger().Write() << search_space_with_buckets.size() << " " << search_space_with_buckets[node].size() ;
        
        if (StallAtNode<false>(node, target_distance, query_heap))
        {
            return;
        }
        
        RelaxOutgoingEdges<false>(node, target_distance, query_heap);
    }
    
    template <bool forward_direction>
    inline void
    RelaxOutgoingEdges(const NodeID node, const EdgeWeight distance, QueryHeap &query_heap) const
    {
        for (auto edge : super::facade->GetAdjacentEdgeRange(node))
        {
            const auto &data = super::facade->GetEdgeData(edge);
            const bool direction_flag = (forward_direction ? data.forward : data.backward);
            if (direction_flag)
            {
                const NodeID to = super::facade->GetTarget(edge);
                const int edge_weight = data.distance;
                const int to_distance = distance + edge_weight;
                
                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_distance, node);
                }
                // Found a shorter Path -> Update distance
                else if (to_distance < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to).parent = node;
                    query_heap.DecreaseKey(to, to_distance);
                }
            }
        }
    }
    
    // Stalling
    template <bool forward_direction>
    inline bool StallAtNode(const NodeID node, const EdgeWeight distance, QueryHeap &query_heap)
    const
    {
        for (auto edge : super::facade->GetAdjacentEdgeRange(node))
        {
            const auto &data = super::facade->GetEdgeData(edge);
            const bool reverse_flag = ((!forward_direction) ? data.forward : data.backward);
            if (reverse_flag)
            {
                const NodeID to = super::facade->GetTarget(edge);
                const int edge_weight = data.distance;
                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                if (query_heap.WasInserted(to))
                {
                    if (query_heap.GetKey(to) + edge_weight < distance)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
};


#endif
