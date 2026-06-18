#!/bin/bash

# Destination directory (docs folder)
dest_dir="docs/"

# Ensure the destination directory exists
lanelet2_core_dir="lanelet2_core"
lanelet2_examples_dir="lanelet2_examples"
lanelet2_io_dir="lanelet2_io"
lanelet2_maps_dir="lanelet2_maps"
lanelet2_matching_dir="lanelet2_matching"
lanelet2_projection_dir="lanelet2_projection"
lanelet2_python_dir="lanelet2_python"
lanelet2_routing_dir="lanelet2_routing"
lanelet2_traffic_rules_dir="lanelet2_traffic_rules"
lanelet2_validation_dir="lanelet2_validation"

cp "README.md" "$dest_dir/index.md"

cp "$lanelet2_core_dir/README.md" "$dest_dir/lanelet2_core.md"
cp -r "$lanelet2_core_dir/doc" "$dest_dir/lanelet2_core"

cp "$lanelet2_examples_dir/README.md" "$dest_dir/lanelet2_examples.md"
cp "$lanelet2_io_dir/README.md" "$dest_dir/lanelet2_io.md"
cp "$lanelet2_maps_dir/README.md" "$dest_dir/lanelet2_maps.md"
cp "$lanelet2_matching_dir/README.md" "$dest_dir/lanelet2_matching.md"
cp "$lanelet2_projection_dir/README.md" "$dest_dir/lanelet2_projection.md"
cp -r "$lanelet2_projection_dir/doc" "$dest_dir/lanelet2_projection"
cp "$lanelet2_python_dir/README.md" "$dest_dir/lanelet2_python.md"

cp "$lanelet2_routing_dir/README.md" "$dest_dir/lanelet2_routing.md"
cp -r "$lanelet2_routing_dir/doc" "$dest_dir/lanelet2_routing"

cp "$lanelet2_traffic_rules_dir/README.md" "$dest_dir/lanelet2_traffic_rules.md"
cp "$lanelet2_validation_dir/README.md" "$dest_dir/lanelet2_validation.md"

echo "Documentation files copied to the docs folder."
