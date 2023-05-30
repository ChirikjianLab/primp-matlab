function df = df_vect(x, f, k, Nr, dim)
% df_vect Uses high order finite difference method to compute numerical
% derivatives along arbitrary dimension of an array
% Uses Frohnberg's method to compute coefficents
%
% Inputs
%   x: elements in domain of known function values, should be a 1-D array
%   f: known function values, f(x), i.e. your input array
%   k: order of the derivative you want to take. If you want to compute the
%      first derivative, k = 1, the second derivative, k = 2, etc.
%   Nr: About half of the window size, i.e. how many points in front and
%       behind you want to consider when computing the derivaitve. This
%       value can vary based on how finely sampled your data is, however
%       Nr = 3 or Nr = 4 usually works well
%   dim: the dimension you want to derive along (usually your temporal or
%        spatial dimension)
%
% Author
%   Tommy Mitchel, tmitchel@jhu.edu, 2018

sz0 = size(f);
f = reshape(f, [prod(sz0)/sz0(dim), sz0(dim)]);

df = zeros(size(f));

for j = 1:Nr
    df(:, j) = sum( fdcoeffF(k, x(j), x(1:Nr+j)).* f(:, 1:Nr+j), 2);
end

for i = Nr+1:sz0(dim)-Nr
    df(:, i) = sum( fdcoeffF(k, x(i), x(i-Nr:i+Nr)).* f(:, i-Nr:i+Nr), 2);
end

for j = sz0(dim) + 1 - Nr:sz0(dim)
    df(:, j) = sum( fdcoeffF(k, x(j), x(j-Nr: end)).* f(:, j-Nr: end), 2);
end

df = reshape(df, sz0);