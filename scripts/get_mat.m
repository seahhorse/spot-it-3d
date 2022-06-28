function [matrix] = get_mat(filename)
% Convert CSV file of detections for a given camera into a 3D matrix
% Input: Name of CSV file
% Output: 3D matrix with the following dimenations:
%   Rows (1) = frame number / time
%   Cols (2) = X (row 1) and Y (row 2) coordinates
%   Depth (3) = Detected IDs

T=readtable(filename);
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

% csv format: frame number, class id, confidence,
% ... x centroid, y centroid, width, height, target id
matrix = zeros(2,1,size(IDs,2));
for row = 1:size(arr,1)
    frame = arr(row,1);
    ID = arr(row,8);
    x = arr(row,4);
    y = arr(row,5);
    ID_pos = find(IDs==ID);
    matrix(1,frame,ID_pos) = x;
    matrix(2,frame,ID_pos) = y;
end

end

