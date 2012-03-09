load 'home_raw.mat'
load 'home_mapping.mat'
data = zeros(0,size(home,2));
dput = 0;
for ii = 1:length(home)
    indx = find(home(ii,3) == home_mapping(:,1));
    if indx
        dput = dput + 1;
        data(dput, :) = home(ii,:);
        data(dput, 3) = home_mapping(indx, 2);
    end
end

save('home.mat', 'data');