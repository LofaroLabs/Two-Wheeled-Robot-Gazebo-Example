#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    public: virtual ~SystemGUI()
    {
      if (this->userCam)
        this->userCam->EnableSaveFrame(false);
    }

    public: virtual void Load(int /*_argc*/, char ** /*_argv*/)
    {
    }

    private: virtual void Init()
    {
      printf("1\n");
      // Get a pointer to the active user camera
      this->userCam = gui::get_active_camera();

      printf("2\n");
      // Enable saving frames
      this->userCam->EnableSaveFrame(true);

      printf("3\n");
      // Specify the path to save frames into
      this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
      printf("4\n");
    }

    private: rendering::UserCameraPtr userCam;
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
