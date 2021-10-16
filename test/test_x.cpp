#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl_resources/config.h"
#include "test_fcl_utility.h"

using BVHModel = fcl::BVHModel<fcl::OBBRSSd>;
using Vector3 = fcl::Vector3<double>;
using Transform3 = fcl::Transform3<double>;
using CollisionObject = fcl::CollisionObject<double>;
using CollisionRequest = fcl::CollisionRequest<double>;
using CollisionResult = fcl::CollisionResult<double>;
using ContinuousCollisionRequest = fcl::ContinuousCollisionRequest<double>;
using ContinuousCollisionResult = fcl::ContinuousCollisionResult<double>;

int main(int argc, char* argv[]) {
  auto verts1 =
      std::vector<Vector3>{Vector3(-1.0, -1.0, 3.0), Vector3(1.0, -1.0, 3.0),
                           Vector3(-1.0, 1.0, 3.0),  Vector3(1.0, 1.0, 3.0),
                           Vector3(-1.0, -1.0, 4.0), Vector3(1.0, -1.0, 4.0),
                           Vector3(-1.0, 1.0, 4.0),  Vector3(1.0, 1.0, 4.0)};
  auto verts2 = std::vector<Vector3>{
      Vector3(-10.0, -10.0, 0.0), Vector3(10.0, -10.0, 0.0),
      Vector3(-10.0, 10.0, 0.0),  Vector3(10.0, 10.0, 0.0),
      Vector3(-10.0, -10.0, 0.5), Vector3(10.0, -10.0, 0.5),
      Vector3(-10.0, 10.0, 0.5),  Vector3(10.0, 10.0, 0.5)};

  auto faces = std::vector<fcl::Triangle>{
      fcl::Triangle(3, 1, 0), fcl::Triangle(3, 0, 2), fcl::Triangle(0, 1, 5),
      fcl::Triangle(0, 5, 4), fcl::Triangle(1, 3, 7), fcl::Triangle(1, 7, 5),
      fcl::Triangle(3, 2, 6), fcl::Triangle(3, 6, 7), fcl::Triangle(2, 0, 4),
      fcl::Triangle(2, 4, 6), fcl::Triangle(4, 5, 7), fcl::Triangle(4, 7, 6),
  };

  auto bvh1 = std::make_shared<BVHModel>();
  bvh1->beginModel(8, 12);
  bvh1->addSubModel(verts1, faces);
  bvh1->endModel();

  auto bvh2 = std::make_shared<BVHModel>();
  bvh2->beginModel(8, 12);
  bvh2->addSubModel(verts2, faces);
  bvh2->endModel();

  auto tf11 = Transform3();
  tf11.setIdentity();
  tf11.translation()[0] = 1.0;
  tf11.translation()[1] = 0.0;
  tf11.translation()[2] = -1.0;

  auto tf12 = Transform3();
  tf12.setIdentity();
  tf12.translation()[0] = 1.0;
  tf12.translation()[1] = 0.0;
  tf12.translation()[2] = -2.0;

  auto tf21 = Transform3();
  tf21.setIdentity();
  tf21.translation()[0] = -2.0;
  tf21.translation()[1] = 0.0;
  tf21.translation()[2] = 0.0;

  auto tf22 = Transform3();
  tf22.setIdentity();
  tf22.translation()[0] = 2.0;
  tf22.translation()[1] = 0.0;
  tf22.translation()[2] = 1.5;

  const auto* obj1 = new CollisionObject(bvh1, tf11);
  const auto* obj2 = new CollisionObject(bvh2, tf21);

  auto request = ContinuousCollisionRequest(
      100, 1e-4, fcl::CCDMotionType::CCDM_TRANS, fcl::GJKSolverType::GST_INDEP,
      fcl::CCDSolverType::CCDC_CONSERVATIVE_ADVANCEMENT);

  auto result = ContinuousCollisionResult();
  auto ret = fcl::continuousCollide(obj1, tf12, obj2, tf22, request, result);
  std::cout << "is_collide: " << result.is_collide << "\n";
  std::cout << "time_of_contact: " << result.time_of_contact << "\n";

  return 0;
}
