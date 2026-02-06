function result = ocv_soc_lookup(ocv_table, query, direction)
% ==========================================================================
%  OCV_SOC_LOOKUP - Interpolasi OCV <-> SOC dari lookup table
% ==========================================================================
%  Menggunakan interpolasi linear (interp1) untuk konversi antara:
%    - SOC -> OCV (direction = 'soc2ocv')
%    - OCV -> SOC (direction = 'ocv2soc')
%
%  Input:
%    ocv_table  - struct dengan field .SOC (%) dan .OCV (V)
%    query      - nilai yang ingin di-lookup (skalar atau vektor)
%    direction  - 'soc2ocv' atau 'ocv2soc'
%
%  Output:
%    result - nilai hasil interpolasi (same size as query)
%
%  Contoh:
%    ocv = ocv_soc_lookup(ocv_table, 50, 'soc2ocv')   % SOC=50% -> OCV
%    soc = ocv_soc_lookup(ocv_table, 3.7, 'ocv2soc')   % OCV=3.7V -> SOC
% ==========================================================================

SOC_data = ocv_table.SOC;  % SOC dalam persen (0-100)
OCV_data = ocv_table.OCV;  % OCV dalam Volt

switch lower(direction)
    case 'soc2ocv'
        % SOC (%) -> OCV (V)
        % Clamp input SOC ke range valid
        query_clamped = max(min(query, max(SOC_data)), min(SOC_data));
        result = interp1(SOC_data, OCV_data, query_clamped, 'linear', 'extrap');

    case 'ocv2soc'
        % OCV (V) -> SOC (%)
        % Pastikan OCV monoton naik untuk inverse lookup yang benar
        % Jika tidak monoton, gunakan unique values
        [OCV_unique, idx_unique] = unique(OCV_data, 'stable');
        SOC_unique = SOC_data(idx_unique);

        % Sort ascending by OCV untuk interp1
        [OCV_sorted, sort_idx] = sort(OCV_unique);
        SOC_sorted = SOC_unique(sort_idx);

        % Clamp input OCV ke range valid
        query_clamped = max(min(query, max(OCV_sorted)), min(OCV_sorted));
        result = interp1(OCV_sorted, SOC_sorted, query_clamped, 'linear', 'extrap');

        % Clamp output SOC ke 0-100%
        result = max(0, min(100, result));

    otherwise
        error('ocv_soc_lookup:InvalidDirection', ...
            'Direction harus ''soc2ocv'' atau ''ocv2soc''. Diterima: %s', direction);
end

end
