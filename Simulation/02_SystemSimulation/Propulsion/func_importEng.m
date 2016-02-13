function engine = func_importEng(fid)
 line = fgetl(fid);
 headerflag = false;
 counter = 1;
  while(ischar(line))
      if(line(1) == ';' || isempty(line))
        line = fgetl(fid);
        continue
      end
      if(headerflag == false)
        headerflag = true;
        tokens = strsplit(line,' ','CollapseDelimiters',true);
        if(length(tokens) ~= 7)
          error('not enough header fields in .eng file')
        end
        engine.name = tokens{1};
        engine.diamger = str2double(tokens{2});
        engine.length = str2double(tokens{3});
        engine.delays = str2double(strsplit(tokens{4},'-'));
        engine.propWeight = str2double(tokens{5});
        engine.totalWeight = str2double(tokens{6});
        engine.manufacturer = tokens{7};
      else
        xy = str2double(strsplit(line,' ','CollapseDelimiters',true));
        engine.time(counter) = xy(1);
        engine.thrust(counter) = xy(2);
        if(engine.thrust(counter) == 0)
          engine.burntime = engine.time(counter);
          break
        end
        counter = counter +1;
      end
      line = fgetl(fid);
  end
end