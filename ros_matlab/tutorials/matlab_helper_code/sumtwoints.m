function response = sumtwoints(~, request, response)
% This service callback function will access the request data structure
% request.A
% request.B 
% And add them together.
% It will place the output on the 
% request.Sum output
% 
% Inputs:
%   ~:          associated with the serivce server object
%   request:    input data struc set by service type
%   response:   output data struc set by service type
%
% Note: the stucture is case-sensitive. Do not use request.a or request.b
% or request.sum.

    response.Sum = request.A + request.B;  % Perform the addition
end