#ifndef _HRI_ACTION_TRIGGER_H_
#define _HRI_ACTION_TRIGGER_H_

namespace gazebo
{
  namespace physics
  {
    enum TriggerState
      {
	T_RELEASE,

	T_FIRE,

	T_ON,

	T_OFF
      };

    template <typename T>
    class ActionTrigger
    {
    public: ActionTrigger() {this->state = TriggerState::T_OFF;};

    public: virtual ~ActionTrigger() {};

    public: TriggerState GetTriggerState() {return this->state;};

    public: T GetValue() {return this->value;};

    public: virtual void TriggerProcess() = 0;

    protected: TriggerState state;

    protected: T value;
    };

    template <typename T>
    using ActionTriggerPtr = boost::shared_ptr<ActionTrigger<T> >;
  }
}

#endif
