#include "validators/mapping/CurvatureTooBig.h"
#include "ValidatorFactory.h"
#include "lanelet2_core/geometry/Point.h"
#include<iostream>

namespace lanelet {
namespace validation {
namespace {
    RegisterMapValidator<CurvatureTooBigChecker> reg1;
}  // namespace



Issues CurvatureTooBigChecker::operator()(const lanelet::LaneletMap& map) {
    Issues issues;
    for (auto lanelet_it = map.laneletLayer.begin();lanelet_it!=map.laneletLayer.end();lanelet_it++) {
        auto left_bound_2d = utils::to2D((*lanelet_it).leftBound());
        auto right_bound_2d = utils::toHybrid(utils::to2D((*lanelet_it).rightBound()));
        if(left_bound_2d.size() >= 3){
            double x0, y0, x1, y1, x2, y2, dx1, dy1, ddx1, ddy1, denom, curvature;
            for(size_t i=1; i<left_bound_2d.size()-1;++i){
                x0 = left_bound_2d[i - 1].x();
                y0 = left_bound_2d[i - 1].y();
                x1 = left_bound_2d[i].x();
                y1 = left_bound_2d[i].y();
                x2 = left_bound_2d[i + 1].x();
                y2 = left_bound_2d[i + 1].y();
                dx1 = 0.5 * (x2 - x0);
                dy1 = 0.5 * (y2 - y0);
                ddx1 = x2 - 2.0 * x1 + x0;
                ddy1 = y2 - 2.0 * y1 + y0;
                // compute "curvature"
                denom = pow(dx1 * dx1 + dy1 * dy1, 3.0 / 2.0);
                if (fabs(denom) < 1e-20)
                    denom = 1e-20;
                curvature = (ddy1 * dx1 - dy1 * ddx1) / denom;
                if(fabs(curvature) > 0.5){
                    issues.emplace_back(
                            Severity::Warning, Primitive::Lanelet, lanelet_it->id(),
                            "Curvature at point " + std::to_string(left_bound_2d[i].id())
                            + " is too big. This can lead to further problems");
                }
            }
        }
        if(right_bound_2d.size() >= 3){
            double x0, y0, x1, y1, x2, y2, dx1, dy1, ddx1, ddy1, denom, curvature;
            for(size_t i=1; i<right_bound_2d.size()-1;++i){
                x0 = right_bound_2d[i - 1].x();
                y0 = right_bound_2d[i - 1].y();
                x1 = right_bound_2d[i].x();
                y1 = right_bound_2d[i].y();
                x2 = right_bound_2d[i + 1].x();
                y2 = right_bound_2d[i + 1].y();
                dx1 = 0.5 * (x2 - x0);
                dy1 = 0.5 * (y2 - y0);
                ddx1 = x2 - 2.0 * x1 + x0;
                ddy1 = y2 - 2.0 * y1 + y0;
                // compute "curvature"
                denom = pow(dx1 * dx1 + dy1 * dy1, 3.0 / 2.0);
                if (fabs(denom) < 1e-20)
                    denom = 1e-20;
                curvature = (ddy1 * dx1 - dy1 * ddx1) / denom;
                if(fabs(curvature) > 0.5){
                    issues.emplace_back(
                            Severity::Warning, Primitive::Lanelet, lanelet_it->id(),
                            "Curvature at point " + std::to_string(right_bound_2d[i].id())
                            + " is too big. This can lead to further problems");
                }
            }
        }
    }
    return issues;
}

}  // namespace validation
}  // namespace lanelet

