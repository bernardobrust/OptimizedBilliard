module PlotUtils

using Printf

function maybe_convert_png(out_svg::String)
    converter = Sys.which("rsvg-convert")
    if converter === nothing
        @warn "rsvg-convert is not installed; leaving the generated SVG without a PNG copy" out_svg
        return
    end
    out_png = replace(out_svg, ".svg" => ".png")
    run(`$converter -f png -d 150 -p 150 -o $out_png $out_svg`)
    println("Generated: $out_svg and $out_png")
end

# Color palette for solutions
const COLOR_SIMPLE   = "#2E7D32" # Dark green
const COLOR_HOMOTOPY = "#C2185B" # Deep pink/magenta
const COLOR_DE_0TH   = "#1976D2" # Blue
const COLOR_DE_EULER = "#00897B" # Teal
const COLOR_DE_RK4   = "#E65100" # Orange

"""
Generate an SVG line plot natively in Julia and optionally convert to PNG.
"""
function plot_line(
    title::String,
    x_label::String,
    y_label::String,
    x_vals::Vector{Int},
    series::Dict{String, Vector{Float64}},
    colors::Dict{String, String},
    out_svg::String
)
    w = 800
    h = 520
    pad_left = 90
    pad_right = 160
    pad_top = 70
    pad_bottom = 70

    plot_w = w - pad_left - pad_right
    plot_h = h - pad_top - pad_bottom

    all_y = Float64[]
    for (k, v) in series
        append!(all_y, v)
    end
    min_y = 0.0
    max_y = maximum(all_y) * 1.15

    # Tick calculation
    min_x = minimum(x_vals)
    max_x = maximum(x_vals)

    x_to_px(x) = pad_left + (x - min_x) / (max_x - min_x) * plot_w
    y_to_px(y) = pad_top + plot_h - (y - min_y) / (max_y - min_y) * plot_h

    io = IOBuffer()
    println(io, """<svg xmlns="http://www.w3.org/2000/svg" width="$w" height="$h" viewBox="0 0 $w $h" font-family="-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif">""")
    println(io, """  <rect width="100%" height="100%" fill="#FFFFFF"/>""")
    println(io, """  <text x="$(w/2)" y="38" text-anchor="middle" font-size="20" font-weight="bold" fill="#212121">$title</text>""")

    # Gridlines and Y-ticks
    n_ticks = 5
    for i in 0:n_ticks
        y_val = min_y + i * (max_y - min_y) / n_ticks
        py = y_to_px(y_val)
        println(io, """  <line x1="$pad_left" y1="$py" x2="$(pad_left + plot_w)" y2="$py" stroke="#E0E0E0" stroke-width="1" stroke-dasharray="4,4"/>""")
        println(io, """  <text x="$(pad_left - 12)" y="$(py + 4)" text-anchor="end" font-size="12" fill="#616161">$(round(Int, y_val))</text>""")
    end

    # X-ticks
    for x in x_vals
        px = x_to_px(x)
        println(io, """  <line x1="$px" y1="$pad_top" x2="$px" y2="$(pad_top + plot_h)" stroke="#E0E0E0" stroke-width="1" stroke-dasharray="4,4"/>""")
        println(io, """  <line x1="$px" y1="$(pad_top + plot_h)" x2="$px" y2="$(pad_top + plot_h + 6)" stroke="#424242" stroke-width="1.5"/>""")
        println(io, """  <text x="$px" y="$(pad_top + plot_h + 24)" text-anchor="middle" font-size="13" fill="#424242">$x</text>""")
    end

    # Axis lines
    println(io, """  <line x1="$pad_left" y1="$pad_top" x2="$pad_left" y2="$(pad_top + plot_h)" stroke="#424242" stroke-width="1.5"/>""")
    println(io, """  <line x1="$pad_left" y1="$(pad_top + plot_h)" x2="$(pad_left + plot_w)" y2="$(pad_top + plot_h)" stroke="#424242" stroke-width="1.5"/>""")

    # Axis Labels
    println(io, """  <text x="$(pad_left + plot_w / 2)" y="$(h - 16)" text-anchor="middle" font-size="14" font-weight="600" fill="#212121">$x_label</text>""")
    println(io, """  <text transform="rotate(-90)" x="$(-pad_top - plot_h / 2)" y="28" text-anchor="middle" font-size="14" font-weight="600" fill="#212121">$y_label</text>""")

    # Draw series lines and points
    names = collect(keys(series))
    sort!(names)
    for name in names
        pts = series[name]
        col = get(colors, name, "#000000")

        # Path
        path_str = ""
        for (idx, x) in enumerate(x_vals)
            px = x_to_px(x)
            py = y_to_px(pts[idx])
            cmd = idx == 1 ? "M" : "L"
            path_str *= "$cmd $(round(px, digits=1)) $(round(py, digits=1)) "
        end
        println(io, """  <path d="$path_str" fill="none" stroke="$col" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>""")

        # Circles on points
        for (idx, x) in enumerate(x_vals)
            px = x_to_px(x)
            py = y_to_px(pts[idx])
            println(io, """  <circle cx="$(round(px, digits=1))" cy="$(round(py, digits=1))" r="4.5" fill="$col" stroke="#FFFFFF" stroke-width="1.5"/>""")
        end
    end

    # Legend on the right
    leg_x = pad_left + plot_w + 20
    leg_y = pad_top + 10
    println(io, """  <rect x="$leg_x" y="$leg_y" width="135" height="$(length(names) * 26 + 16)" rx="6" fill="#F8F9FA" stroke="#E0E0E0" stroke-width="1"/>""")
    for (idx, name) in enumerate(names)
        col = get(colors, name, "#000000")
        item_y = leg_y + 14 + (idx - 1) * 26
        println(io, """  <line x1="$(leg_x + 12)" y1="$item_y" x2="$(leg_x + 32)" y2="$item_y" stroke="$col" stroke-width="3"/>""")
        println(io, """  <circle cx="$(leg_x + 22)" cy="$item_y" r="4" fill="$col"/>""")
        println(io, """  <text x="$(leg_x + 40)" y="$(item_y + 4)" font-size="12" font-weight="500" fill="#333333">$name</text>""")
    end

    println(io, "</svg>")

    write(out_svg, String(take!(io)))
    maybe_convert_png(out_svg)
end

"""
Generate an SVG grouped bar chart natively in Julia and convert to PNG.
"""
function plot_grouped_bar(
    title::String,
    y_label::String,
    categories::Vector{String},
    series_names::Vector{String},
    data::Dict{String, Vector{Float64}}, # series_name => [val_cat1, val_cat2, ...]
    colors::Dict{String, String},
    out_svg::String
)
    w = 880
    h = 520
    pad_left = 90
    pad_right = 160
    pad_top = 70
    pad_bottom = 80

    plot_w = w - pad_left - pad_right
    plot_h = h - pad_top - pad_bottom

    all_y = Float64[]
    for (k, v) in data
        append!(all_y, v)
    end
    min_y = 0.0
    max_y = maximum(all_y) * 1.2

    y_to_px(y) = pad_top + plot_h - (y - min_y) / (max_y - min_y) * plot_h

    io = IOBuffer()
    println(io, """<svg xmlns="http://www.w3.org/2000/svg" width="$w" height="$h" viewBox="0 0 $w $h" font-family="-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif">""")
    println(io, """  <rect width="100%" height="100%" fill="#FFFFFF"/>""")
    println(io, """  <text x="$(w/2)" y="38" text-anchor="middle" font-size="20" font-weight="bold" fill="#212121">$title</text>""")

    # Gridlines and Y-ticks
    n_ticks = 5
    for i in 0:n_ticks
        y_val = min_y + i * (max_y - min_y) / n_ticks
        py = y_to_px(y_val)
        println(io, """  <line x1="$pad_left" y1="$py" x2="$(pad_left + plot_w)" y2="$py" stroke="#E0E0E0" stroke-width="1" stroke-dasharray="4,4"/>""")
        println(io, """  <text x="$(pad_left - 12)" y="$(py + 4)" text-anchor="end" font-size="12" fill="#616161">$(round(Int, y_val))</text>""")
    end

    # Group geometry
    n_cats = length(categories)
    group_width = plot_w / n_cats
    n_series = length(series_names)
    bar_width = min(group_width * 0.75 / n_series, 28.0)
    total_bars_width = bar_width * n_series

    for (c_idx, cat) in enumerate(categories)
        group_center = pad_left + (c_idx - 0.5) * group_width
        start_x = group_center - total_bars_width / 2

        # Category label
        println(io, """  <text x="$group_center" y="$(pad_top + plot_h + 24)" text-anchor="middle" font-size="13" font-weight="600" fill="#424242">$cat</text>""")

        for (s_idx, s_name) in enumerate(series_names)
            val = data[s_name][c_idx]
            col = get(colors, s_name, "#000000")
            bx = start_x + (s_idx - 1) * bar_width
            by = y_to_px(val)
            bh = (pad_top + plot_h) - by

            # Bar rectangle
            println(io, """  <rect x="$(round(bx, digits=1))" y="$(round(by, digits=1))" width="$(round(bar_width - 2, digits=1))" height="$(round(bh, digits=1))" rx="3" fill="$col"/>""")
            # Value label above bar
            println(io, """  <text x="$(round(bx + (bar_width - 2)/2, digits=1))" y="$(round(by - 5, digits=1))" text-anchor="middle" font-size="10" font-weight="bold" fill="#424242">$(round(Int, val))</text>""")
        end
    end

    # Axis lines
    println(io, """  <line x1="$pad_left" y1="$pad_top" x2="$pad_left" y2="$(pad_top + plot_h)" stroke="#424242" stroke-width="1.5"/>""")
    println(io, """  <line x1="$pad_left" y1="$(pad_top + plot_h)" x2="$(pad_left + plot_w)" y2="$(pad_top + plot_h)" stroke="#424242" stroke-width="1.5"/>""")

    # Axis labels
    println(io, """  <text transform="rotate(-90)" x="$(-pad_top - plot_h / 2)" y="28" text-anchor="middle" font-size="14" font-weight="600" fill="#212121">$y_label</text>""")

    # Legend
    leg_x = pad_left + plot_w + 16
    leg_y = pad_top + 10
    println(io, """  <rect x="$leg_x" y="$leg_y" width="135" height="$(n_series * 26 + 16)" rx="6" fill="#F8F9FA" stroke="#E0E0E0" stroke-width="1"/>""")
    for (s_idx, name) in enumerate(series_names)
        col = get(colors, name, "#000000")
        item_y = leg_y + 14 + (s_idx - 1) * 26
        println(io, """  <rect x="$(leg_x + 12)" y="$(item_y - 6)" width="12" height="12" rx="2" fill="$col"/>""")
        println(io, """  <text x="$(leg_x + 32)" y="$(item_y + 4)" font-size="12" font-weight="500" fill="#333333">$name</text>""")
    end

    println(io, "</svg>")

    write(out_svg, String(take!(io)))
    maybe_convert_png(out_svg)
end

end # module PlotUtils
