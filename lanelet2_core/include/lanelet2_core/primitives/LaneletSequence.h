#pragma once
#include <utility>

#include "lanelet2_core/primitives/CompoundLineString.h"
#include "lanelet2_core/primitives/CompoundPolygon.h"
#include "lanelet2_core/primitives/Lanelet.h"

namespace lanelet {

/**
 * @brief Common data management class for LaneletSequences
 *
 * This object is only for internal data management and should not be used
 * directly.
 *
 * LaneletSequence merely has a shared_ptr to this object and forwards all
 * calls to this object.
 *
 * Because calculating a polygon and a centerline is expensive, the results are
 * cached by this object.
 */
class LaneletSequenceData : public boost::noncopyable {
 public:
  using iterator =  // NOLINT
      internal::ReverseAndForwardIterator<ConstLanelets::const_iterator>;
  /**
   * @brief Constructs a new, valid LaneletData object
   * @see ConstLanelet::ConstLanelet
   */
  explicit LaneletSequenceData(ConstLanelets lanelets);

  const CompoundLineString3d& leftBound() const { return leftBound3d(); }
  const CompoundLineString3d& leftBound3d() const { return leftBound_; }
  CompoundLineString2d leftBound2d() const { return utils::to2D(leftBound_); }

  const CompoundLineString3d& rightBound() const { return rightBound3d(); }
  const CompoundLineString3d& rightBound3d() const { return rightBound_; }
  CompoundLineString2d rightBound2d() const { return utils::to2D(rightBound3d()); }
  const ConstLanelets& lanelets() const { return lanelets_; }

  /**
   * @brief computes the centerline. Result is cached
   * @return centerline object. Can not be modified.
   * @see ConstLanelet::centerline
   */
  CompoundLineString3d centerline() const;

  /**
   * @brief Get the bounding polygon around all lanelets. Result is cached.
   * @return a polygon object. Can not be modified.
   * @see ConstLanelet::polygon
   */
  CompoundPolygon3d polygon() const;

  iterator begin(bool inverted) const noexcept {
    return inverted ? iterator(lanelets_.rbegin()) : iterator(lanelets_.begin());
  }

  iterator end(bool inverted) const noexcept {
    return inverted ? iterator(lanelets_.rend()) : iterator(lanelets_.end());
  }

 private:
  const ConstLanelets lanelets_;  // NOLINT
  //!< Cached data
  const CompoundLineString3d leftBound_;                      // NOLINT
  const CompoundLineString3d rightBound_;                     // NOLINT
  mutable std::shared_ptr<CompoundLineString3d> centerline_;  //!< combined centerline
  mutable std::shared_ptr<CompoundPolygon3d> polygon_;        //!< combined polygon
};

/**
 * @brief A collection of Lanelets
 * @see ConstLanelet
 *
 * A LaneletSequence is a collection of Lanelets that behaves like
 * like one single Lanelet. Because of that, it can be passed to most functions
 * expecting a normal lanelet.
 *
 * The contents of a LaneletSequence can not be modified. Modifications have to
 * be done to the Lanelets directly.
 *
 * If
 */
class LaneletSequence {
 public:
  using DataType = LaneletSequenceData;
  using TwoDType = void;
  using ThreeDType = void;
  using HybridType = void;
  using ConstType = LaneletSequence;
  using MutableType = void;
  using iterator = LaneletSequenceData::iterator;  // NOLINT

  /**
   * @brief Construct from a vector of ConstLineString3d
   * @param ls objects to construct from. The order will also be the order
   * of the linestrings.
   */
  LaneletSequence(ConstLanelets ls = ConstLanelets())  // NOLINT
      : data_{std::make_shared<LaneletSequenceData>(std::move(ls))} {}

  /**
   * @brief Create a compound lanelet from other compound lanelets
   * @param ls vector of compound lanelets
   */
  explicit LaneletSequence(const LaneletSequences& ls)
      : LaneletSequence(utils::concatenate(ls, [](const auto& e) { return e.lanelets(); })) {}

  LaneletSequence(LaneletSequenceDataConstPtr data, bool inverted) : data_{std::move(data)}, inverted_{inverted} {}

  /**
   * @brief returns out if this is an inverted lanelets.
   * @return true if inverted
   *
   * Inverted LaneletSequences behave differently from normal lanelets in that
   * they internally interpret their inverted left bound as right bound and vice
   * versa.
   */
  bool inverted() const { return inverted_; }

  //! Returns wether this holds any lanelets
  bool empty() const noexcept { return data_->lanelets().empty(); }

  //! Returns number of lanelets
  size_t size() const noexcept { return data_->lanelets().size(); }

  /**
   * @brief returns the respective inverted LaneletSequence
   *
   * Both lanelets (this and the inverted one) share the same data.
   * Inversion only requires copying a shared ptr.
   */
  LaneletSequence invert() const { return {constData(), !inverted()}; }

  //! get the combined left bounds of all lanelets
  CompoundLineString3d leftBound() const { return leftBound3d(); }
  //! get the left bound in 2d. To be used over leftBound where geometric calculations are required.
  CompoundLineString2d leftBound2d() const { return utils::to2D(leftBound3d()); }
  CompoundLineString3d leftBound3d() const {
    return inverted() ? constData()->rightBound().invert() : constData()->leftBound();
  }

  //! get the combined right bounds of all lanelets
  CompoundLineString3d rightBound() const { return rightBound3d(); }
  //! get the right bound in 2d. To be used over rightBound where geometric calculations are required.
  CompoundLineString2d rightBound2d() const { return utils::to2D(rightBound3d()); }
  //! get the combined right bounds of all lanelets
  CompoundLineString3d rightBound3d() const {
    return inverted() ? constData()->leftBound().invert() : constData()->rightBound();
  }

  //! get a list of all regulatory elements that affect one of the lanelets
  RegulatoryElementConstPtrs regulatoryElements() const;

  /**
   * @brief get all regulatory elements of type RegElemT
   * @return list of pointers to regulatory elements in the LaneletSequence
   */
  template <typename RegElemT>
  std::vector<std::shared_ptr<const RegElemT>> regulatoryElementsAs() const {
    return utils::concatenate(lanelets(),
                              [](const auto& elem) { return elem.template regulatoryElementsAs<RegElemT>(); });
  }

  /**
   * @brief get the centerline of this lanelet
   *
   * Note the computation of the centerline is expensive, but as the result is
   * cached, repeated calls to this functions are cheap.
   *
   * The returned centerline is only guarateed to be within the the lanelet, but
   * not always perfecly in the middle of it. However, if your lanelet is more
   * or
   * less monotonic, this is negligible.
   *
   * The start (and similarly the end point) will always be direcly in the
   * middle between the first left and right points.
   */
  CompoundLineString3d centerline() const { return centerline3d(); }
  CompoundLineString3d centerline3d() const {
    return inverted() ? constData()->centerline().invert() : constData()->centerline();
  }
  CompoundLineString2d centerline2d() const { return utils::to2D(centerline3d()); }

  /**
   * @brief returns the surface covered by the lanelets as 3-dimensional
   * polygon.
   * The result of the computation is cached, therefore foo(polygon().begin(),
   * polygon.end()) provides valid iterators.
   *
   * Note that many geometry functions are only available for a 2d-polygon.
   */
  CompoundPolygon3d polygon3d() const { return constData()->polygon(); }

  /**
   * @brief returns the surface covered by the lanelets as 2-dimensional
   * polygon.
   */
  CompoundPolygon2d polygon2d() const { return CompoundPolygon2d(polygon3d()); }

  /**
   * @brief returns the lanelets that are part of this object
   * @return vector of lanelets
   */
  ConstLanelets lanelets() const { return inverted() ? utils::invert(data_->lanelets()) : data_->lanelets(); }

  //! Returns iterator to the first lanelet (or to the last, if inverted)
  iterator begin() const noexcept { return data_->begin(inverted()); }

  //! Returns iterator to the last lanelet (or to the first, if inverted)
  iterator end() const noexcept { return data_->end(inverted()); }

  //! Access an individual lanelet
  const ConstLanelet& operator[](size_t idx) const {
    assert(idx < data_->lanelets().size());
    return *std::next(begin(), idx);
  }

  /**
   * @brief returns the ids of all lanelets in order
   * @return list of ids
   */
  Ids ids() const {
    return utils::transform(lanelets(), [](const auto& ls) { return ls.id(); });
  }

  /**
   * @brief returns the internal data on the linestrings managed by this object
   */
  LaneletSequenceDataConstPtr constData() const { return data_; }

  /**
   * @brief call this function if one of the contained Lanelets was modified to
   * give this
   * object the opportunity to update its cache.
   */
  void resetCache() { data_ = std::make_shared<LaneletSequenceData>(data_->lanelets()); }

 private:
  std::shared_ptr<const LaneletSequenceData> data_;  //!< internal data
  bool inverted_{false};                             //!< Flag that indicates this lanelet is inverted
};

namespace utils {
/**
 * @brief returns true if element of a  primitive has a matching Id
 * @param ll the element holding other primitives
 * @param id id to look for
 * @return true if the primitive has such an element
 *
 * This function does not look for the id of the element, only its members
 * Works for linestrings and polylines.
 * A similar implementation exists for regulatory elements and lanelets.
 */
inline bool has(const LaneletSequence& ll, Id id) { return has(ll.leftBound(), id) || has(ll.rightBound(), id); }
}  // namespace utils

inline bool operator==(const LaneletSequence& lhs, const LaneletSequence& rhs) {
  return lhs.lanelets() == rhs.lanelets();
}
inline bool operator!=(const LaneletSequence& lhs, const LaneletSequence& rhs) { return !(rhs == lhs); }

inline std::ostream& operator<<(std::ostream& stream, const LaneletSequence& obj) {
  auto ids = obj.ids();
  stream << "[";
  if (!ids.empty()) {
    stream << "ids: ";
    std::copy(ids.begin(), ids.end(), std::ostream_iterator<Id>(stream, " "));
  }
  if (obj.inverted()) {
    stream << ", inverted";
  }
  return stream << "]";
}
}  // namespace lanelet
