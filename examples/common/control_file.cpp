#include "control_file.hpp"

#include <fstream>
#include <ostream>
#include <sstream>
#include <stdexcept>

#include "afcs.hpp"

namespace examples {

std::vector<ControlRow> read_controls(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::string line;
    int n = 0;
    // The header is the first line that isn't a comment.
    while (std::getline(in, line)) {
        ++n;
        if (!line.starts_with('#')) {
            break;
        }
    }
    const bool with_afcs = line.starts_with(kControlHeaderAfcs);
    if (!with_afcs && !line.starts_with(kControlHeader)) {
        throw std::runtime_error(path + ": expected header " + kControlHeader + " or " +
                                 kControlHeaderAfcs);
    }
    std::vector<ControlRow> rows;
    while (std::getline(in, line)) {
        ++n;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream s(line);
        ControlRow r;
        char c1 = 0;
        char c2 = 0;
        char c3 = 0;
        char c4 = 0;
        if (!(s >> r.t >> c1 >> r.d_collective >> c2 >> r.d_lat >> c3 >> r.d_lon >> c4 >>
              r.d_pedal) ||
            c1 != ',' || c2 != ',' || c3 != ',' || c4 != ',') {
            throw std::runtime_error(path + ":" + std::to_string(n) + ": bad row");
        }
        if (with_afcs) {
            char c5 = 0;
            std::string text;
            if (!(s >> c5 >> text) || c5 != ',') {
                throw std::runtime_error(path + ":" + std::to_string(n) + ": bad row (afcs)");
            }
            try {
                r.afcs = parse_afcs(text, '+');
            } catch (const std::runtime_error& e) {
                throw std::runtime_error(path + ":" + std::to_string(n) + ": " + e.what());
            }
            r.has_afcs = true;
        }
        if (!rows.empty() && r.t < rows.back().t) {
            throw std::runtime_error(path + ":" + std::to_string(n) + ": time goes backwards");
        }
        rows.push_back(r);
    }
    return rows;
}

void write_control_row(std::ostream& out, const ControlRow& r) {
    const auto old = out.precision(17);
    out << r.t << ',' << r.d_collective << ',' << r.d_lat << ',' << r.d_lon << ',' << r.d_pedal;
    if (r.has_afcs) {
        out << ',' << afcs_text(r.afcs, '+');
    }
    out << '\n';
    out.precision(old);
}

eyas::Controls ControlSchedule::at(const eyas::Controls& start, double t) noexcept {
    eyas::Controls c = start;
    if (rows_ == nullptr) {
        return c;
    }
    while (next_ < rows_->size() && (*rows_)[next_].t <= t) {
        ++next_;
    }
    if (const ControlRow* row = current(); row != nullptr) {
        c.collective += row->d_collective;
        c.lat_cyclic += row->d_lat;
        c.lon_cyclic += row->d_lon;
        c.pedal += row->d_pedal;
        if (row->has_afcs) {
            c.afcs = row->afcs;
        }
    }
    return c;
}

const ControlRow* ControlSchedule::current() const noexcept {
    return rows_ == nullptr || next_ == 0 ? nullptr : &(*rows_)[next_ - 1];
}

}  // namespace examples
