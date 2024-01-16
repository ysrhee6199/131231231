#include "shared_carlalib.h"
#include "CarlaRGBCamera.hpp"
#include "CarlaRadar.hpp"


/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}/// Save a semantic segmentation image to disk converting to CityScapes palette.

  
  static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
}



int main(int argc, const char *argv[]) {
    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);    
    std::mt19937_64 rng((std::random_device())());    
    auto client = cc::Client(host, port);
    client.SetTimeout(40s);    
    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';  

    // Load a Town04.
    auto town_name = "/Game/Carla/Maps/Town04";
    std::cout << "Loading world: " << town_name << std::endl;
    auto world = client.LoadWorld(town_name);    
    auto blueprint_library = world.GetBlueprintLibrary(); 
    //Get a Trailer
    auto trailer = blueprint_library->Filter("trailer");
    auto blueprint_trailer = RandomChoice(*trailer, rng);
    // Get a Truck blueprint.
    auto vehicles = blueprint_library->Filter("dafxf");
    auto blueprint = RandomChoice(*vehicles, rng);  

    // Find a valid spawn point for trailer.
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng); // TODO :: 선택으로 변경
    
    transform.location.x = 11.60f;
    transform.location.y = -8.62f;
    transform.location.z = 1.0f;
    transform.rotation.roll = 0.0f;
    transform.rotation.pitch = 0.0f;
    transform.rotation.yaw= -90.22f;
    
    // Spawn the trailer
    auto actor_trailer = world.SpawnActor(blueprint_trailer, transform);
    std::cout << "Spawned " << actor_trailer->GetDisplayId() << '\n';
    auto trailer_ = boost::static_pointer_cast<cc::Vehicle>(actor_trailer);    

    // Find a valid spawn point for truck
    transform.location += 5.2f * transform.GetForwardVector();    

    // Spawn the truck
    auto actor = world.SpawnActor(blueprint, transform);
    std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);    

    // Set autopilot
//    vehicle->SetAutopilot(true);    

    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);  

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
   // auto node = std::make_shared<CarlaRGBCameraPublisher>(blueprint_library,actor,world);
    //auto node_radar = std::make_shared<CarlaRadarPublisher>(blueprint_library,actor,world);
   // executor.add_node(node);
   // executor.add_node(node_radar);





/***********************FV1*********************/

    //Get a Trailer
    auto trailer_fv1 = blueprint_library->Filter("trailer");
    auto blueprint_trailer_fv1 = RandomChoice(*trailer, rng);
    // Get a Truck blueprint.
    auto vehicles_fv1 = blueprint_library->Filter("dafxf");
    auto blueprint_fv1 = RandomChoice(*vehicles_fv1, rng);  
    
    transform.location.x = 11.90f;
    transform.location.y = 10.32f;
    transform.location.z = 1.0f;
    transform.rotation.roll = 0.0f;
    transform.rotation.pitch = 0.0f;
    transform.rotation.yaw= -90.22f;

    // Spawn the trailer
    auto actor_trailer_fv1 = world.SpawnActor(blueprint_trailer_fv1, transform);
    std::cout << "Spawned " << actor_trailer_fv1->GetDisplayId() << '\n';
    auto trailer_fv1_ = boost::static_pointer_cast<cc::Vehicle>(actor_trailer_fv1);    

    // Find a valid spawn point for truck
    transform.location += 5.2f * transform.GetForwardVector();    

    // Spawn the truck
    auto actor_fv1 = world.SpawnActor(blueprint_fv1, transform);
    std::cout << "Spawned " << actor_fv1->GetDisplayId() << '\n';
    auto vehicle_fv1 = boost::static_pointer_cast<cc::Vehicle>(actor_fv1);    

    // Set autopilot
  //  vehicle_fv1->SetAutopilot(true);  

 //   vehicle->SetAutopilot(true);
    auto node_fv1 = std::make_shared<CarlaRGBCameraPublisher>(blueprint_library,actor_fv1,world);
    auto node_radar_fv1 = std::make_shared<CarlaRadarPublisher>(blueprint_library,actor_fv1,world);
    executor.add_node(node_fv1);
    executor.add_node(node_radar_fv1);
       
/***********************FV1*********************/










    executor.spin();




     









    

    rclcpp::shutdown(); 






    // Remove actors from the simulation.
      
  vehicle->Destroy();
    trailer_->Destroy();
    vehicle_fv1->Destroy();
    trailer_fv1_->Destroy();


    std::cout << "Actors destroyed." << std::endl;  

    return 0;
}