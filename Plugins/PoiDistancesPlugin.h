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

template <class DataFacadeT> class PoiDistancesPlugin final : public BasePlugin
{
private:
    std::unique_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;
    
public:
    explicit PoiDistancesPlugin(DataFacadeT *facade) : descriptor_string("poitable"), facade(facade)
    {
        search_engine_ptr = osrm::make_unique<SearchEngine<DataFacadeT>>(facade);
    }
    
    virtual ~PoiDistancesPlugin() {}
    
    const std::string GetDescriptor() const final { return descriptor_string; }

void HandleRequest(const RouteParameters &route_parameters, http::Reply &reply) final
{
// check number of parameters
if (1 != route_parameters.coordinates.size())
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
unsigned max_locations = 1;
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

TIMER_START(poi_distance_table);
std::shared_ptr<std::unordered_map<NodeID, EdgeWeight>> result_table =
search_engine_ptr->poi_distance_table(phantom_node_vector[0].front(), 5000);
TIMER_STOP(poi_distance_table);

if (!result_table)
{
    reply = http::Reply::StockReply(http::Reply::badRequest);
    return;
}
JSON::Object json_object;
JSON::Array json_array;
for ( PhantomNode pnode : facade->GetPoisPhantomNodeList() )
{
    if( result_table->find( pnode.forward_node_id) != result_table->end() ) {
        JSON::Object row;
        row.values["node_id"] = pnode.info_osm_id ;
        row.values["distance"] = (*result_table)[pnode.forward_node_id] ;
        json_array.values.push_back(row);
    }
}
json_object.values["distance_table"] = json_array;
JSON::render(reply.content, json_object);
}

private:
std::string descriptor_string;
DataFacadeT *facade;
};


#endif
