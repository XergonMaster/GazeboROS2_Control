#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
    class FixedJointVelocityPlugin : public ModelPlugin
    {
    public:
        FixedJointVelocityPlugin() : ModelPlugin()
        {
            std::cout << "FixedJointVelocityPlugin loaded" << std::endl;
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the model pointer for convenience
            this->model = _parent;

            // Default values for parameters
            this->jointName = "joint";
            this->velocity = 1.0;

            // Check for SDF parameters
            if (_sdf->HasElement("jointName"))
                this->jointName = _sdf->Get<std::string>("jointName");

            if (_sdf->HasElement("velocity"))
                this->velocity = _sdf->Get<double>("velocity");

            // Get the joint
            this->joint = this->model->GetJoint(this->jointName);
            if (!this->joint)
            {
                gzerr << "Joint " << this->jointName << " not found in model " << this->model->GetName() << std::endl;
                return;
            }

            // Connect to the update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&FixedJointVelocityPlugin::OnUpdate, this));

            std::cout << "FixedJointVelocityPlugin loaded successfully" << std::endl;
        }

        void OnUpdate()
        {
            // Set the joint velocity
            this->joint->SetVelocity(0, this->velocity);
        }

    private:
        physics::ModelPtr model;
        physics::JointPtr joint;
        event::ConnectionPtr updateConnection;
        std::string jointName;
        double velocity;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FixedJointVelocityPlugin)
}
