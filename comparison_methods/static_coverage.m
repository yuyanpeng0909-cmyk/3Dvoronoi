function positions = static_coverage(initial_positions, ~, ~)
% static_coverage - 静态均匀覆盖：智能体位置不随时间变化
%
% 输入:
%   initial_positions: n x 3 初始位置
%   ~ (t): 未使用
%   ~ (params): 未使用
%
% 输出:
%   positions: n x 3，与输入相同

    positions = initial_positions;
end
