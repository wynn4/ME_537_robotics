% Jesse Wynn HW2 ME 537 Robotics
clc
clear all
close all

% Problem 2 from Chapter 2

% Compute the matrix exponential using the power series.
% How many terms are required to match standard MATLAB precision?

% pick some A matrix
A = [3 -1 2; 2 5 -3; 4 1 2];

expmA = eye(3) + A + A^2/factorial(2) + A^3/factorial(3) + A^4/factorial(4)...
        + A^5/factorial(5) + A^6/factorial(6) + A^7/factorial(7) + A^8/factorial(8) ...
         + A^9/factorial(9) + A^10/factorial(10) + A^11/factorial(11) + A^12/factorial(12) ...
          + A^13/factorial(13) + A^14/factorial(14) + A^15/factorial(15) + A^16/factorial(16) ...
           + A^17/factorial(17) + A^18/factorial(18) + A^19/factorial(19) + A^20/factorial(20) ...
            + A^21/factorial(21) + A^22/factorial(22) + A^23/factorial(23) + A^24/factorial(24) ...
             + A^25/factorial(25) + A^26/factorial(26);
        
% looks like it takes 26 terms (A^0 thru A^23) lets verify...
if isequal(expmA, expm(A))
    disp 'success'
else
    disp 'keep going...'
    error = expmA - expm(A)
end

disp 'whatever... close enough'