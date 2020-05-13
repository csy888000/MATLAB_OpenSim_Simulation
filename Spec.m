classdef Spec
    properties
        id
        timestep_limit
    end
    methods
        function sp = Spec()
            sp.id = 0;
            sp.timestep_limit = 300;
        end
    end
end