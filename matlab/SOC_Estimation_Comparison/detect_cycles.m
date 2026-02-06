function cycles = detect_cycles(time, current, min_duration)
% ==========================================================================
%  DETECT_CYCLES - Deteksi siklus charge/discharge dari data arus
% ==========================================================================
%  Mengidentifikasi transisi siklus berdasarkan perubahan tanda arus.
%
%  Logika:
%    - Scan data arus: identifikasi kapan arus berubah tanda secara konsisten
%    - Perubahan tanda yang bertahan > min_duration detik = transisi siklus
%    - Setiap segmen antara transisi = 1 siklus (charge atau discharge)
%
%  Konvensi arus: positif = discharge, negatif = charge
%
%  Input:
%    time         - vektor waktu (detik), Nx1
%    current      - vektor arus (Ampere), Nx1
%    min_duration - durasi minimum transisi (detik), default 30
%
%  Output:
%    cycles - struct array dengan field:
%      .start_idx  - index awal siklus
%      .end_idx    - index akhir siklus
%      .start_time - waktu awal (detik)
%      .end_time   - waktu akhir (detik)
%      .type       - 'charge' atau 'discharge'
%      .duration   - durasi siklus (detik)
%
%  Catatan:
%    Jika hanya ada 1 siklus (misal hanya discharge), ECC tetap berjalan
%    tapi tanpa update parameter antar-siklus.
% ==========================================================================

if nargin < 3
    min_duration = 30;  % Default: 30 detik threshold
end

N = length(time);

% Tentukan tanda arus per sampel (1 = discharge, -1 = charge, 0 = rest)
% Gunakan threshold kecil untuk menghindari noise di sekitar nol
I_threshold = 0.1;  % Ampere - arus di bawah ini dianggap rest
sign_current = zeros(N, 1);
sign_current(current > I_threshold) = 1;    % discharge
sign_current(current < -I_threshold) = -1;  % charge

% Cari transisi yang bertahan lebih dari min_duration
% Strategi: sliding window untuk konfirmasi transisi
transitions = [1];  % Selalu mulai dari index 1
current_sign = sign_current(1);

% Cari sign dominan awal (skip rest periods di awal)
for k = 1:N
    if sign_current(k) ~= 0
        current_sign = sign_current(k);
        break;
    end
end

k = 2;
while k <= N
    % Cek apakah tanda berubah
    if sign_current(k) ~= 0 && sign_current(k) ~= current_sign
        % Kandidat transisi - verifikasi durasi
        new_sign = sign_current(k);
        trans_start = k;

        % Hitung durasi di mana tanda baru bertahan secara dominan
        consistent_duration = 0;
        check_idx = k;
        count_new = 0;
        count_total = 0;

        while check_idx <= N && (time(check_idx) - time(trans_start)) < min_duration * 2
            if sign_current(check_idx) ~= 0
                count_total = count_total + 1;
                if sign_current(check_idx) == new_sign
                    count_new = count_new + 1;
                end
            end
            check_idx = check_idx + 1;
        end

        % Transisi valid jika > 70% sampel dalam window memiliki tanda baru
        % DAN durasi window mencapai min_duration
        if count_total > 0 && (count_new / count_total) > 0.7 && ...
           check_idx > trans_start && (time(min(check_idx, N)) - time(trans_start)) >= min_duration
            transitions(end+1) = trans_start; %#ok<AGROW>
            current_sign = new_sign;
            k = check_idx;
            continue;
        end
    end
    k = k + 1;
end

% Tambahkan akhir data sebagai batas terakhir
transitions(end+1) = N;

% Buat struct array siklus
n_cycles = length(transitions) - 1;
cycles = struct('start_idx', {}, 'end_idx', {}, 'start_time', {}, ...
                'end_time', {}, 'type', {}, 'duration', {});

for c = 1:n_cycles
    idx_start = transitions(c);
    idx_end = transitions(c+1);

    % Tentukan tipe siklus berdasarkan arus rata-rata
    avg_current = mean(current(idx_start:idx_end));

    cycles(c).start_idx = idx_start;
    cycles(c).end_idx = idx_end;
    cycles(c).start_time = time(idx_start);
    cycles(c).end_time = time(idx_end);
    cycles(c).duration = time(idx_end) - time(idx_start);

    if avg_current > 0
        cycles(c).type = 'discharge';
    else
        cycles(c).type = 'charge';
    end
end

% Print ringkasan deteksi siklus
fprintf('--- Deteksi Siklus ---\n');
fprintf('  -> Total siklus terdeteksi: %d\n', n_cycles);
for c = 1:n_cycles
    fprintf('  -> Siklus %d: %s (%.0f s - %.0f s, durasi %.0f s)\n', ...
        c, cycles(c).type, cycles(c).start_time, cycles(c).end_time, cycles(c).duration);
end
fprintf('\n');

end
