function B = generate_B(n, dt)
    syms t
    % 1. Define the new basis up to t^7
    b = [1, t, t^2, t^3, t^4, t^5, t^6, t^7];

    % 2. Compute derivatives up to 6th order
    b_d    = diff(b, t);      % velocity
    b_dd   = diff(b_d, t);    % acceleration
    b_ddd  = diff(b_dd, t);   % jerk
    b_dddd = diff(b_ddd, t);  % snap
    b_5    = diff(b_dddd, t); % crackle
    b_6    = diff(b_5, t);    % pop

    sub_col_sz = length(b);           % = 8 now
    num_segments = n - 1;
    total_cols = num_segments * sub_col_sz;
    total_lines = 2 * num_segments + (n - 2) * 6 + 4; % same logic, but 6 continuity constraints now
    B = zeros(total_lines, total_cols);

    row_idx = 1;

    % 3. Position constraints at start and end of each segment
    for i = 1:num_segments
        B(row_idx, (i-1)*sub_col_sz + (1:sub_col_sz)) = double(subs(b, t, (i-1)*dt));
        B(row_idx+1, (i-1)*sub_col_sz + (1:sub_col_sz)) = double(subs(b, t, i*dt));
        row_idx = row_idx + 2;
    end

    % 4. Continuity constraints at internal waypoints (velocity to 6th derivative)
    derivs = {b_d, b_dd, b_ddd, b_dddd, b_5, b_6};
    for d = 1:length(derivs)
        for i = 1:(n - 2)
            B(row_idx, (i-1)*sub_col_sz + (1:sub_col_sz)) = double(subs(derivs{d}, t, i*dt));
            B(row_idx, i*sub_col_sz + (1:sub_col_sz)) = -double(subs(derivs{d}, t, i*dt));
            row_idx = row_idx + 1;
        end
    end

    % 5. Initial and final derivative constraints (velocity, acceleration and jerk)
    B(row_idx, 1:sub_col_sz) = double(subs(b_d, t, 0));
    row_idx = row_idx + 1;
    B(row_idx, end-sub_col_sz+1:end) = double(subs(b_d, t, n*dt));
    row_idx = row_idx + 1;
    B(row_idx, 1:sub_col_sz) = double(subs(b_dd, t, 0));
    row_idx = row_idx + 1;
    B(row_idx, end-sub_col_sz+1:end) = double(subs(b_dd, t, n*dt));
    row_idx = row_idx + 1;
	B(row_idx, 1:sub_col_sz) = double(subs(b_ddd, t, 0));
	row_idx = row_idx + 1;
	B(row_idx, end-sub_col_sz+1:end) = double(subs(b_ddd, t, n*dt));
end

