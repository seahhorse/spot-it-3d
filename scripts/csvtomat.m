% Convert CSV file of detections for a given camera into a 3D matrix
% Matrix dimensions:
% Rows (1) = frame number / time
% Cols (2) = X (row 1) and Y (row 2) coordinates
% Depth (3) = Detected IDs

T=readtable('output.csv');
arr=table2array(T);

% get list of IDs
IDs = 0;
count = 1;
for i = 1:size(arr,1)
    ID_found = find(IDs == arr(i,2));
    if isempty(ID_found)
        IDs(count) = arr(i,2);
        count = count + 1;
    end
end

result = zeros(2,1,size(IDs,2));
for row = 1:size(arr,1)
    frame = arr(row,1);
    ID = arr(row,2);
    x = arr(row,3);
    y = arr(row,4);
    ID_pos = find(IDs==ID);
    result(1,frame,ID_pos) = x;
    result(2,frame,ID_pos) = y;
end

save('matrix.mat','result')

toc;