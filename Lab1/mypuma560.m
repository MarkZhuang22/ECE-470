function myrobot = mypuma560(DH)
    % Get the dimensions of the DH parameter matrix
    DH_dim = size(DH);
    
    % Initialize an empty Link array 
    L = Link.empty(DH_dim(1), 0);
    
    % Loop through each row in the DH table to create the Link objects
    for i = 1:DH_dim(1)
        L(i) = Link('d',DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4));
    end
    
    % Create a SerialLink object
    myrobot = SerialLink(L, 'name', 'puma560');
end
