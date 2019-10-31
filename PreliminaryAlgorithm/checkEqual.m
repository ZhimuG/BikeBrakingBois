function [ ] = checkEqual( actVal, expVal, tol, checkID)
%checkEqual The function checkEqual checks whether or not the actual value
%actVal is equal to the expected value expVal within the specified
%tolerance tol. If no tolerance is specified it checks whether the actual
%value actVal is exactly equal to the expected value expVal. If the actual
%and expected objects are arrays, checkEqual checks whether all of the
%array elements meet the condition, element-wise.

% Check whether the objects actVal and expVal are numeric. If either of
% them are not, throw an error.
if ~(isnumeric(actVal))
    
    error('Actual value is non-numeric');
    
elseif ~(isnumeric(expVal))
    
    error('Expected value is non-numeric');
    
end

% Check whether the numeric objects actVal and expVal are of the same
% numeric class. If they are not, throw a warning.
if ~isequal(class(actVal), class(expVal))
    
    warningMsg = ['Actual and expected values are not of the same data '...
        'type: the actual value is of the class ', class(actVal), ... 
        ' and the expected value is of the class ', class(expVal), '.'];
    warning(warningMsg);

end

%Find any NaNs in the expected and actual values and replace them with
%zeros. 

% This assumes that NaNs are equal!

expVal(isnan(expVal) == true) = 0;
actVal(isnan(actVal) == true) = 0;

%Find any Infs in the expected and actual values and replace them with -1
%or 1 depending on the sign of the Inf

expVal(isinf(expVal) == true) = sign(expVal(isinf(expVal) == true));
actVal(isinf(actVal) == true) = sign(actVal(isinf(actVal) == true));


% Check whether a tolerance has been specified.
   
if exist('tol', 'var') && ~isempty(tol)
    
    %assertionMsg = ['Actual value and expected value are not equal '...
    %    'within the specified tolerance'];
    % If a tolerance is specified, check whether the actual and expected
    % values are equal to within this tolerance.
    checkLogicalArr = abs(actVal - expVal) <= tol;
    checkLogical = isempty(checkLogicalArr(checkLogicalArr == false));
    %assert(checkLogical, assertionMsg)
    
else
    
    %assertionMsg = ['Actual value and expected value are not exactly '...
    %        'equal'];
    % If a tolerance is not specified, check whether the actual and
    % expected values are exactly equal.
    checkLogicalArr = actVal == expVal;
    checkLogical = isempty(checkLogicalArr(checkLogicalArr == false));
    %assert(checkLogical, assertionMsg)
    
    % If either the actual value, expected value or both are floating point
    % numbers, throw a warning.
    if isfloat(actVal) || isfloat(expVal)
        warningMsg = ['The actual and/or expected values are floating '...
            'point numbers, typically a tolerance is required when '...
            'comparing floating point numbers. Try using a tolerance '...
            'of 1e-12, the floating point number representation error.'];
        warning(warningMsg);
    end
    
end

if ~exist('checkID', 'var')
    checkID = num2str(1);
end

if checkLogical
    passMsg = ['Check ', num2str(checkID), ': Passed!\n'];
    fprintf(passMsg);
else
    failMsg = ['Check ', num2str(checkID), ': Failed!\n'];
    fprintf(failMsg);
end

end

