# -*- coding: utf-8 -*-
"""
Script to generate the complete ScholarRead-style bilingual reflowed markdown document
for the Nature 2024 paper:
"Fast ground-to-air transition with avian-inspired multifunctional legs"
"""
import os

output_path = "reference/Fast_ground-to-air_transition_bilingual.md"
print(f"Generating bilingual document at {output_path}...")

# We will build the markdown content in modules
