#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include <sstream>
#include <string>

using namespace std;

class FibonacciAction {
protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line. Otherwise
  // strange error occurs.
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;
  string action_name_;

  // Create messages that are used to publish feedback/result.
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

public:
  FibonacciAction(string name)
      : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1),
            false),
        action_name_(name) {
    as_.start();
  }

  ~FibonacciAction(void) {}

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal) {
    ros::Rate r(1);
    bool success = true;

    // Push back the seeds for the fibonacci sequence.
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // Publish info to the console for the user.
    stringstream msg;
    msg << action_name_ << ": Executing, creating fibonacci sequence of order "
        << goal->order << " with seeds " << feedback_.sequence[0] << ", "
        << feedback_.sequence[1];
    ROS_INFO(msg.str());

    // Start executing the action.
    for (int i = 1; i <= goal->order; i++) {
      // Check that preempt has not been requested by the client.
      if (as_.isPreecptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());

        // Set the action state to preempted.
        as_.setPreempted();
        success = false;
        break;
      }

      feedback_.sequence.push_back(feedback_.sequence[i] +
                                   feedback_.sequence[i - 1]);

      // Publish the feedback.
      as_.publishFeedback(feedback_);

      // This sleep is not necessary, the sequence is computed at 1 Hz
      // for demonstration purposes.
      r.sleep();
    }

    if (success) {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());

      // Set the action state to succeeded.
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}
