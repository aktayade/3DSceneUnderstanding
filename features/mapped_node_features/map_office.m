load 'office_raw.mat'
load 'office_mapping.mat'
data = zeros(0,size(office,2));
dput = 0;
for ii = 1:length(office)
    indx = find(office(ii,3) == office_mapping(:,1));
    if indx
        dput = dput + 1;
        data(dput, :) = office(ii,:);
        data(dput, 3) = office_mapping(indx, 2);
        if data(dput,3) == max(office_mapping(:,2))
            data(dput,3) = 0;
        end
    end
end

save('office.mat', 'data');