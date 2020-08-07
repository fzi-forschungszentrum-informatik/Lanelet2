#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include "lanelet2_examples/internal/ExampleHelpers.h"

// we want assert statements to work in release mode
#undef NDEBUG

void part1BasicRegulatoryElements();
void part2HandlingRegulatoryElements();
void part3AddingNewRegulatoryElements();

int main() {
  // this tutorial show you how to use regulatory elements in a lanelet map. This tutorial is divided in 4 parts:
  part1BasicRegulatoryElements();
  part2HandlingRegulatoryElements();
  part3AddingNewRegulatoryElements();
  return 0;
}

void part1BasicRegulatoryElements() {
  using namespace lanelet;
  // lanelet2 defines some basic regulatory elements such as traffic lights and speed limits. This tutorial shows you
  // how they work and interact with lanelets. We could do the same thing with areas, because the interface is
  // identical.

  // traffic lights are created from a linestring that shows a traffic light and optionally a stop line.
  LineString3d trafficLight = examples::getLineStringAtY(1);
  trafficLight.attributes()[AttributeName::Type] = AttributeValueString::TrafficLight;

  // this creates our traffic light. Regelems are passed around as shared pointers.
  RegulatoryElementPtr trafficLightRegelem = lanelet::TrafficLight::make(utils::getId(), {}, {trafficLight});

  // actually, traffic light regelem is not a RegulatoryElement (this is the base class), but a TrafficLight.
  // we could cast it back to get the actual type. But first lets see, how to get this into a lanelet:
  Lanelet lanelet = examples::getALanelet();
  lanelet.addRegulatoryElement(trafficLightRegelem);  // thats it.
  assert(lanelet.regulatoryElements().size() == 1);

  // to get the regulatory element back, we can either get it like this
  RegulatoryElementPtr regelem = lanelet.regulatoryElements()[0];

  // but we can also ask the lanelet to give it to use with its actual type using regulatoryElementAs:
  assert(lanelet.regulatoryElementsAs<SpeedLimit>().empty());  // no speed limits
  std::vector<TrafficLight::Ptr> trafficLightRegelems = lanelet.regulatoryElementsAs<TrafficLight>();
  assert(trafficLightRegelems.size() == 1);
  TrafficLight::Ptr tlRegelem = trafficLightRegelems.front();          // here it is with its correct type.
  assert(tlRegelem->constData() == trafficLightRegelem->constData());  // they are actually the same.

  // from traffic lights we can directly get the relevant lights and the stop line (we didnt set one, but we could).
  // since traffic lights can either be a polygon or a linestring, we get an object that represents both.
  LineStringOrPolygon3d theLight = tlRegelem->trafficLights().front();
  assert(theLight == trafficLight);

  // we can also modify it, and since regulatory element data is shared, this also affects the lanelet that holds it
  tlRegelem->setStopLine(examples::getLineStringAtY(2));
  assert(!!tlRegelem->stopLine());

  // there are much more regulatory elements, but they all work basically in the same way as shown here.
}

void part2HandlingRegulatoryElements() {
  using namespace lanelet;
  // actually, the parts of the interface we just showed to you are mostly only for convenience. The internal strucure
  // is a bit more complicated. To show that, we use a GenericRegulatoryElement that can be used to model any rule.
  // However, it should not be used in practice, because it generic structure makes it too hard to interpret it.
  GenericRegulatoryElement regelem(utils::getId());

  // to the generic regulatory elements we can add any primitive (point, linestring, lanelet, area) with any role:
  Lanelet lanelet = examples::getALanelet();
  regelem.addParameter(RoleName::Refers, lanelet);
  Point3d point(utils::getId(), 0, 0, 0);
  regelem.addParameter(RoleName::Refers, point);

  // now two different primitives have been added with the same role. Internally they are stored as boost::variants.
  // to read them from the regelem, we have to pass the type we are looking for:
  Points3d pts = regelem.getParameters<Point3d>(RoleName::Refers);
  assert(!pts.empty() && pts.front() == point);

  // this interface could be used add more things with more nonsense role names. But that would be hard to interpret.
  // For that reason, the implementations of regulatory elements provide an interface that gives less opportunity for
  // abuse.
}

// as an example, we create a new regulatory element and register it with lanelet2. It is a "LightsOn" regulatory
// element, that tells the vehicle to turn its lights on after passing a specific line
namespace example {
class LightsOn : public lanelet::RegulatoryElement {  // we have to inherit from the abstract regulatoryElement
 public:
  // lanelet2 looks for this string when matching the subtype of a regulatory element to the respective type
  static constexpr char RuleName[] = "lights_on";

  // returns the line where we are supposed to stop
  lanelet::ConstLineString3d fromWhere() const {
    return getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine).front();
  }

 private:
  LightsOn(lanelet::Id id, lanelet::LineString3d fromWhere)
      : RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)} {
    parameters().insert({lanelet::RoleNameString::RefLine, {fromWhere}});
  }

  // the following lines are required so that lanelet2 can create this object when loading a map with this regulatory
  // element
  friend class lanelet::RegisterRegulatoryElement<LightsOn>;
  explicit LightsOn(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data) {}
};

#if __cplusplus < 201703L
constexpr char LightsOn::RuleName[];  // instanciate string in cpp file
#endif
}  // namespace example

namespace {
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<example::LightsOn> reg;
}  // namespace

void part3AddingNewRegulatoryElements() {
  using namespace lanelet;
  // after creating our new class and registering it, we can test if it works. For that we use the
  // RegulatoryElementFactory that is used by Lanelet2_io when loading a map. If we did it right, it should now return
  // a regulatory element of the LightsOn class.

  // for that we create a valid regulatory element data object
  LineString3d fromWhere = examples::getLineStringAtX(1);
  RuleParameterMap rules{{RoleNameString::RefLine, {fromWhere}}};

  RegulatoryElementPtr regelem = RegulatoryElementFactory::create("lights_on", utils::getId(), rules);

  // now we can add it to a lanelet and query for it
  Lanelet lanelet = examples::getALanelet();
  lanelet.addRegulatoryElement(regelem);
  assert(!lanelet.regulatoryElementsAs<example::LightsOn>().empty());
}
