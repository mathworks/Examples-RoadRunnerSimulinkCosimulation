classdef SRE < Simulink.IntEnumType
  enumeration
    Balancing(0)
    Charging(1)
    Driving(2)
  end
  methods (Static)
    function retVal = getDefaultValue()
      retVal = SRE.Balancing;
    end
  end
end 