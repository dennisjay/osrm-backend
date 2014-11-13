//
//  PoiDistancesPlugin.h
//  osrm-backend
//
//  Created by Dennis Jöst on 01.11.14.
//  Copyright (c) 2014 Dennis Jöst. All rights reserved.
//

#ifndef osrm_backend_PoiDistancesPlugin_h
#define osrm_backend_PoiDistancesPlugin_h

#include "BasePlugin.h"

#include "../Algorithms/ObjectToBase64.h"
#include "../DataStructures/JSONContainer.h"
#include "../DataStructures/QueryEdge.h"
#include "../DataStructures/SearchEngine.h"
#include "../Descriptors/BaseDescriptor.h"
#include "../Util/make_unique.hpp"
#include "../Util/StringUtil.h"
#include "../Util/TimingUtil.h"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

template <class DataFacadeT> class OneToManyPlugin final : public BasePlugin
{
private:
    std::unique_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;
    
public:
    explicit OneToManyPlugin(DataFacadeT *facade) : descriptor_string("oneToMany"), facade(facade)
    {
        search_engine_ptr = osrm::make_unique<SearchEngine<DataFacadeT>>(facade);
    }
    
    virtual ~OneToManyPlugin() {}
    
    const std::string GetDescriptor() const final { return descriptor_string; }

void HandleRequest(const RouteParameters &route_parameters, http::Reply &reply) final
{
    // check number of parameters
    if (2 > route_parameters.coordinates.size())
    {
        reply = http::Reply::StockReply(http::Reply::badRequest);
        return;
    }

    RawRouteData raw_route;
    raw_route.check_sum = facade->GetCheckSum();

    if (std::any_of(begin(route_parameters.coordinates),
                end(route_parameters.coordinates),
                [&](FixedPointCoordinate coordinate)
                {
                    return !coordinate.isValid();
                }))
    {
        reply = http::Reply::StockReply(http::Reply::badRequest);
        return;
    }

    for (const FixedPointCoordinate &coordinate : route_parameters.coordinates)
    {
        raw_route.raw_via_node_coordinates.emplace_back(std::move(coordinate));
    }

    const bool checksum_OK = (route_parameters.check_sum == raw_route.check_sum);
    unsigned max_locations =
    std::min(100u, static_cast<unsigned>(raw_route.raw_via_node_coordinates.size()));
    PhantomNodeArray phantom_node_vector(max_locations);
    for (unsigned i = 0; i < max_locations; ++i)
    {
        if (checksum_OK && i < route_parameters.hints.size() &&
            !route_parameters.hints[i].empty())
        {
            PhantomNode current_phantom_node;
            ObjectEncoder::DecodeFromBase64(route_parameters.hints[i], current_phantom_node);
            if (current_phantom_node.isValid(facade->GetNumberOfNodes()))
            {
                phantom_node_vector[i].emplace_back(std::move(current_phantom_node));
                continue;
            }
        }
        facade->IncrementalFindPhantomNodeForCoordinate(raw_route.raw_via_node_coordinates[i],
                                                    phantom_node_vector[i],
                                                    route_parameters.zoom_level,
                                                    1);
    
        BOOST_ASSERT(phantom_node_vector[i].front().isValid(facade->GetNumberOfNodes()));
    }

    TIMER_START(one_to_many);
    std::shared_ptr<std::vector<EdgeWeight>> result_table =
    search_engine_ptr->one_to_many(phantom_node_vector.front().front(), phantom_node_vector);
    TIMER_STOP(one_to_many);
    unsigned ms_used = TIMER_MSEC(one_to_many) ;

    if (!result_table)
    {
        reply = http::Reply::StockReply(http::Reply::badRequest);
        return;
    }
    JSON::Object json_object;
    JSON::Array json_array;
    const unsigned number_of_locations = static_cast<unsigned>(phantom_node_vector.size());
    FixedPointCoordinate src = raw_route.raw_via_node_coordinates[0] ;
    for (unsigned row = 0; row < number_of_locations; ++row)
    {
        JSON::Object json_inner_object ;
        json_inner_object.values["distance"] = result_table->at(row) ;
        json_inner_object.values["lat"] = raw_route.raw_via_node_coordinates[row].lat ;
        json_inner_object.values["lon"] = raw_route.raw_via_node_coordinates[row].lon ;
        json_inner_object.values["air_distance"] =
            FixedPointCoordinate::ApproximateDistance(src, raw_route.raw_via_node_coordinates[row]) ;
        
        json_array.values.push_back(json_inner_object);
    }
    json_object.values["distance_table"] = json_array;
    json_object.values["time_used_ms"] = ms_used ;
    JSON::render(reply.content, json_object);
}

private:
std::string descriptor_string;
DataFacadeT *facade;
};


#endif
