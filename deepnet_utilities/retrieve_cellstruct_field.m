function [ result ] = retrieve_cellstruct_field( cellstruct, field )
%RETRIEVE_CELLSTRUCT_FIELD Utility function to extract a field from a cell
%array containing a single struct per cell

    c = reshape([cellstruct{:}], size(cellstruct));
    result = reshape([c.(field)], size(cellstruct));

end

