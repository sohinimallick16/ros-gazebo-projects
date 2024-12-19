#include <gazebo/gazebo.hh> 

namespace gazebo
{
  // Define a custom WorldPlugin class
  class WorldPluginMyWorld : public WorldPlugin
  {
    public: 
      // Constructor for the plugin
      WorldPluginMyWorld() : WorldPlugin()
      {
        // Print a welcome message to the console when the plugin is loaded
        printf("Welcome to Sohini's World!\n");
      }

      /**
       * @brief Function called when the plugin is loaded into the simulation.
       * @param _world A pointer to the Gazebo simulation world.
       * @param _sdf A pointer to the plugin's SDF configuration.
       */
      public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
      {
      }
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyWorld)
}
