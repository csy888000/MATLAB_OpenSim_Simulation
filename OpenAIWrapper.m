 classdef OpenAIWrapper < rl.env.MATLABEnvironment
  properties
    open_env = py.gym.make('CartPole-v1'); 
  end
  methods
    function this = OpenAIWrapper()
      ObservationInfo = rlNumericSpec([4 1]);
      ObservationInfo.Name = 'CartPoleObservation';
      ObservationInfo.Description = 'Position, Velocity, Angle, VelocityAtTip';
      ActionInfo = rlFiniteSetSpec([0 1]);
      ActionInfo.Name = 'PushingPole';
      this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
    end
    function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
      result = this.open_env.step(int16(Action)); 
      Observation = double(result{1})'; 
      Reward = result{2};
      IsDone = result{3};
      LoggedSignals = [];
    end
    function InitialObservation = reset(this)
      result = this.open_env.reset();
      InitialObservation = double(result)'; 
    end
  end
end 