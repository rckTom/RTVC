classdef dataLogger
  properties
    data;
    variables;
    length;
    currentIndex = 1;
  end
  methods
    function obj = dataLogger(variables,length)
      obj.variables = variables;
      obj.length = length;
      for i = 1:size(obj.variables,1)
        obj.data.(obj.variables{i,2}) = zeros([variables{i,3}(1),variables{i,3}(2)*length]);
      end
      fieldnames(obj.data)
      disp(['reserved ' num2str(sizeof(obj.data)) ' bytes of memory for logging']);
    end
    
    function obj = setup(obj,dt)
  
    end
    
    function obj = log(obj)
      for i=1:size(obj.variables,1)
        obj.data.(obj.variables{i,2})(:,obj.currentIndex*obj.variables{i,3}(2)) = evalin('caller',obj.variables{i,1});
      end
      obj.currentIndex = obj.currentIndex + 1;
    end
    
    function obj = endLogging(obj)
      %Remove trailing zeros if we have not used all space
      if obj.currentIndex < obj.length
        for i = 1:size(obj.variables,1)
          obj.data.(obj.variables{i,2})(:,(obj.currentIndex)*obj.variables{i,3}(2):end) = [];
        end
      end
    end
    
  end

end