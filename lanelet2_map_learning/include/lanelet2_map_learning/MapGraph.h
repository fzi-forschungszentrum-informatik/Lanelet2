#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <map>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

/** @brief Main class of the map learning module that holds the graph information and can be queried.
 *  The MapGraph class is the central object of this module and is initialized with a LaneletMap and TrafficRules.
 *  A graph with all interesting relations will be created for the traffic participant featured in the provided
 * TrafficRules module. The graph can answer queries like "left", "following", "conflicting" lanelets.
 *
 * @note The direction of lanelets matters! 'lanelet' and 'lanelet.invert()' are differentiated since this matters when
 * lanelets are passable in both directions.
 * @note 'adjacent_left' and 'adjacent_right' means that there is a passable lanelet left/right of another passable
 * lanelet, but a lane change is not allowed. */
class MapGraph {
 public:
  using Errors = std::vector<std::string>;                 ///< For the checkValidity function
  using Configuration = std::map<std::string, Attribute>;  ///< Used to provide a configuration
  //! Defined configuration attributes
  static constexpr const char ParticipantHeight[] = "participant_height";

  /** @brief Main constructor with optional configuration.
   *  @param laneletMap Map that should be used to build the graph
   *  @param trafficRules Traffic rules that apply to find passable lanelets
   *  @param config Optional configuration */
  static MapGraphUPtr build(const LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                            const Configuration& config = Configuration());

  //! Similar to the above but for a LaneletSubmap
  static MapGraphUPtr build(const LaneletSubmap& laneletSubmap, const traffic_rules::TrafficRules& trafficRules,
                            const Configuration& config = Configuration());

  //! The graph can not be copied, only moved
  MapGraph() = delete;
  MapGraph(const MapGraph&) = delete;
  MapGraph& operator=(const MapGraph&) = delete;
  MapGraph(MapGraph&& /*other*/) noexcept;
  MapGraph& operator=(MapGraph&& /*other*/) noexcept;
  ~MapGraph();

  /** @brief Determines the relation between two lanelets
   *  @param from Start lanelet
   *  @param to Goal lanelet
   *  @param includeConflicting if false, conflicting lanelets are not considered as relations
   *  @return Relation between the lanelets or nothing if no relation exists. Nothing if at least one of the lanelets
   * is not passable. */
  Optional<RelationType> routingRelation(const ConstLanelet& from, const ConstLanelet& to,
                                         bool includeConflicting = false) const;

  //! Return type neccessary for vertex std::map comparison
  ConstLaneletOrAreas getLaneletEdges(const ConstLanelet& lanelet, bool edgesOut = true) const;

  /** @brief Returns the lanelets that can be reached from this lanelet.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return Lanelets that can be directly reached
   *  @see followingRelations */
  ConstLanelets following(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Returns the lanelets that can be reached from this lanelet and the relation.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return Lanelets can be directly reached
   *  @see following */
  LaneletRelations followingRelations(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Returns the possible previous lanelets of this lanelet.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return All previous lanelets
   *  @see previousRelations */
  ConstLanelets previous(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Returns the possible previous lanelets of this lanelet and the relation.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return Lanelets that could be used to reach this lanelet
   *  @see previous */
  LaneletRelations previousRelations(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Retrieve all reachable left and right lanelets
   *  @param lanelet Start lanelet
   *  @return All left and right lanelets that can be reached, including lanelet, ordered left to right. */
  ConstLanelets besides(const ConstLanelet& lanelet) const;

  /** @brief Get left (routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Left lanelet if it exists. Nothing if it doesn't.
   *  @see adjacentLeft, lefts, adjacentLefts */
  Optional<ConstLanelet> left(const ConstLanelet& lanelet) const;

  /** @brief Get adjacent left (non-routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start laneletfoo
   *  @return Adjacent left lanelet if it exists. Nothing if it doesn't.
   *  @see left, lefts, adjacentLefts */
  Optional<ConstLanelet> adjacentLeft(const ConstLanelet& lanelet) const;

  /** @brief Get right (routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Right lanelet if it exists. Nothing if it doesn't.
   *  @see adjacentRight, rights, adjacentRights */
  Optional<ConstLanelet> right(const ConstLanelet& lanelet) const;

  /** @brief Get adjacent right (non-routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Adjacent right lanelet if it exists. Nothing if it doesn't.
   *  @see right, rights, adjacentRights */
  Optional<ConstLanelet> adjacentRight(const ConstLanelet& lanelet) const;

  /** @brief Get all left (routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Left lanelets if they exists. Empty if they don't.
   *  @see adjacentLeft, left, adjacentLefts */
  ConstLanelets lefts(const ConstLanelet& lanelet) const;

  /** @brief Get all adjacent left (non-routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Adjacent left lanelets if they exists. Empty if they don't.
   *  @see adjacentLeft, left, lefts */
  ConstLanelets adjacentLefts(const ConstLanelet& lanelet) const;

  /** @brief Retrieve all lanelets and relations left of a given lanelet.
   *  @param lanelet Start lanelet
   *  @return All lanelets and relations left of a given lanelet.
   *  @see lefts, adjacentLefts */
  LaneletRelations leftRelations(const ConstLanelet& lanelet) const;

  /** @brief Get all right (routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Right lanelets if they exists. Empty if they don't.
   *  @see adjacentRight, right, adjacentRights */
  ConstLanelets rights(const ConstLanelet& lanelet) const;

  /** @brief Get all adjacent right (non-routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Adjacent right lanelets if they exists. Empty if they don't.
   *  @see adjacentRight, right, rights */
  ConstLanelets adjacentRights(const ConstLanelet& lanelet) const;

  /** @brief Retrieve all lanelets and relations right of a given lanelet.
   *  @param lanelet Start lanelet
   *  @return All lanelets and relations right of a given lanelet.
   *  @see rights, adjacentRights */
  LaneletRelations rightRelations(const ConstLanelet& lanelet) const;

  /** @brief Retrieve all lanelets that are conflicting with the given lanelet.
   *
   *  Conflicting means that their bounding boxes overlap and the height clearance is smaller than the specified
   * "participant_height".
   *  @param laneletOrArea Lanelet/Area to get conflicting lanelets for.
   *  @return All conflicting lanelets. */
  ConstLaneletOrAreas conflicting(const ConstLaneletOrArea& laneletOrArea) const;

  /** @brief Export the internal graph to graphML (xml-based) file format.
   *  @param filename Fully qualified file name - ideally with extension (.graphml)
   *  @param edgeTypesToExclude Exclude the specified relations. E.g. conflicting. Combine them with "|".
      @see exportGraphViz */
  void exportGraphML(const std::string& filename, const RelationType& edgeTypesToExclude = RelationType::None) const;

  /** @brief Export the internal graph to graphViz (DOT) file format.
   *  This format includes coloring of the edges in the graph and bears little more information than graphML export.
   *  @param filename Fully qualified file name - ideally with extension (.gv)
   *  @param edgeTypesToExclude Exclude the specified relations. E.g. conflicting. Combine them with "|". */
  void exportGraphViz(const std::string& filename, const RelationType& edgeTypesToExclude = RelationType::None) const;

  /** @brief An abstract lanelet map holding the information of the graph.
   *  A good way to view the graph since it can be exported using the lanelet2_io module and there can be
   * viewed in tools like JOSM. Each lanelet is represented by a point at the center of gravity of the lanelet.
   * Relations are linestrings between points representing lanelets.
   *  @param includeAdjacent Also include adjacent (non-routable) relations
   *  @param includeConflicting Also include conflicting relations
   *  @return LaneletMap with the requested information */
  LaneletMapPtr getDebugLaneletMap(bool includeAdjacent = false, bool includeConflicting = false) const;

  /**
   * @brief Returns a submap that contains all lanelets and areas within this graph, and nothing else.
   * You can obtain a full map of the graph by calling passabelSubmap()->laneletMap(), which ist a potentially
   * costly operation.
   */
  inline LaneletSubmapConstPtr passableSubmap() const noexcept { return passableLaneletSubmap_; }

  /** @brief LaneletSubmap that includes all passable lanelets and areas.
   *  This map contains all passable lanelets and areas with all primitives (linestrings, points), including regulatory
   * elements and lanelets referenced by them. It can be used to perform spacial queries e.g. for localization. When
   * selecting a lanelet from this map please be aware that the graph may also contain the inverted lanelet.
   *  @return LaneletMap with all passable lanelets and areas
   *
   * This function is deprecated because it was misleading that the map also contained lanelets referenced by regulatory
   * elements and not only the lanelets from the graph.
   */
  [[deprecated("Use passableSubmap to obtain the lanelets and areas within the graph!")]] inline LaneletMapConstPtr
  passableMap() const noexcept {
    return passableSubmap()->laneletMap();
  }

  /** @brief Performs some basic sanity checks.
   *  It is recommended to call this function after the graph has been generated since it can point out some
   * mapping errors.
   *  @throws MapGraphError if an error is found an 'throwOnError' is true
   *  @param throwOnError Decide wheter to throw an exception or just return the errors
   *  @return Possible errors if 'throwOnError' is false. */
  Errors checkValidity(bool throwOnError = true) const;

  /**
   * Constructs the graph. Don't call this directly, use MapGraph::build instead.
   */
  MapGraph(std::unique_ptr<internal::MapGraphGraph>&& graph, lanelet::LaneletSubmapConstPtr&& passableMap);

  friend TensorGraphData getLaneLaneData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                                         const ParametrizationType& paramType, int bezierNPoints);
  friend TensorGraphData getLaneTEData(MapGraphConstPtr localSubmapGraph, const LaneletRepresentationType& reprType,
                                       const ParametrizationType& paramType, int bezierNPoints);

 private:
  //! Documentation to be found in the cpp file.
  std::unique_ptr<internal::MapGraphGraph> graph_;  ///< Wrapper of the graph
  LaneletSubmapConstPtr passableLaneletSubmap_;     ///< Lanelet map of all passable lanelets
};

}  // namespace map_learning
}  // namespace lanelet
