close all
clear all

dire = dir('vel=*alpha=*');
alphaRange = 0:10;
velRange = 5:2.5:30;
for i = 1:length(dire)
  [vel,alpha] = strread(dire(i).name,'vel=%falpha=%f');
  fid = fopen([dire(i).name '/forceCoeffs.dat']);
  data = textscan(fid,'%f\t%f\t%f\t%f\t%f\t%f','HeaderLines',9);
  fclose(fid);
  alphaIndex = find(alphaRange == alpha);
  velIndex = find(velRange == vel);
  results(alphaIndex,velIndex,1) = mean(data{3}(end-4:end));
  results(alphaIndex,velIndex,2) = mean(data{4}(end-4:end));
end

plot(results(:,:,2))
