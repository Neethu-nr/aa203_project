function idx = state_to_idx(S, minimum, scale)
    idx = round((S-minimum)./scale*N);
end
function state = idx_to_state(idx, minimum, scale)
    state = idx.*scale/10 + minimum;
end