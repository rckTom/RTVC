close all
clear all

dir = glob('vel=*alpha=*');
alphaRange = 0:9;
velRange = 5:2.5:30;
for i = 1:length(dir)
  [vel alpha] = strread(dir{i},'vel=%falpha=%f');
  fid = fopen([dir{i} '/forceCoeffs.dat']);
  data = textscan(fid,'%f\t%f\t%f\t%f\t%f\t%f','HeaderLines',9);
  fclose(fid);
  alphaIndex = find(alphaRange == alpha);
  velIndex = find(velRange == vel);
  results(alphaIndex,velIndex,1) = mean(data{3}(end-4:end));
  results(alphaIndex,velIndex,2) = mean(data{4}(end-4:end));
  plot(results(:,:,1),results(:,:,2),'LineWidth',1.5,'-*')
end
