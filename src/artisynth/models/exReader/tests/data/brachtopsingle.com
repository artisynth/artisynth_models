gfx read nodes brachtopsingle brachtopsingle.exnode
gfx read elements brachtopsingle brachtopsingle.exelem

gfx define tessellation tess8 minimum_divisions "8" refinement_factors "1";
gfx modify g_element /brachtopsingle/ lines coordinate coordinates tessellation tess8 LOCAL select_on material blue selected_material default_selected;
# gfx modify g_element /brachtopsingle/ surfaces coordinate coordinates tessellation tess8 LOCAL select_on material default selected_material default_selected render_shaded;

gfx create window 1
gfx edit scene

