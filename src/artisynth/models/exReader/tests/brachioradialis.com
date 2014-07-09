gfx read nodes brachioradialis brachioradialis.exnode
gfx read elements brachioradialis brachioradialis.exelem

gfx modify g_element "brachioradialis" lines material blue
gfx create region "brachsmall"
gfx convert elements source_region "brachioradialis" refinement 2*2*2 convert_trilinear destination_region "brachsmall" fields "coordinates"

gfx create window 1
gfx edit scene

