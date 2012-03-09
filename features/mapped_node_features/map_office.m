load 'office_raw.mat'
load 'office_mapping.mat'
data = zeros(0,size(office,2));
dput = 0;
for ii = 1:length(office)
    indx = find(office(ii,3) == office_mapping);
    if indx
        dput = dput + 1;
        data(dput, :) = office(ii,:);
        data(dput, 3) = indx;
    end
end

save('office.mat', 'data');