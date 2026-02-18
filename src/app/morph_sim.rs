use std::mem;

use image::ImageBuffer;

use crate::app::{SeedColor, SeedPos, preset::Preset};

#[cfg(not(target_arch = "wasm32"))]
use crate::app::preset::UnprocessedPreset;

// const DST_FORCE: f32 = 0.2;
pub fn init_image(sidelen: u32, source: Preset) -> (u32, Vec<SeedPos>, Vec<SeedColor>, Sim) {
    let imgpath = image::ImageBuffer::from_vec(
        source.inner.width,
        source.inner.height,
        source.inner.source_img,
    )
    .unwrap();
    let assignments = source.assignments;

    let (seeds, colors, seeds_n) = init_colors(sidelen, imgpath);
    let mut sim = Sim::new(source.inner.name);
    sim.cells = vec![CellBody::new(0.0, 0.0, 0.0, 0.0, 0.0); seeds_n];

    sim.set_assignments(assignments, sidelen);
    for cell in &mut sim.cells {
        cell.dst_force = 0.13;
    }
    (seeds_n as u32, seeds, colors, sim)
}

#[cfg(not(target_arch = "wasm32"))]
pub fn init_canvas(
    sidelen: u32,
    source: UnprocessedPreset,
) -> (u32, Vec<SeedPos>, Vec<SeedColor>, Sim) {
    use crate::app::calculate::drawing_process::DRAWING_CANVAS_SIZE;
    let imgpath =
        image::ImageBuffer::from_vec(source.width, source.height, source.source_img).unwrap();
    let assignments = (0..(DRAWING_CANVAS_SIZE * DRAWING_CANVAS_SIZE)).collect::<Vec<usize>>();

    let (seeds, colors, seeds_n) = init_colors(sidelen, imgpath);
    let mut sim = Sim::new(source.name);
    sim.cells = vec![CellBody::new(0.0, 0.0, 0.0, 0.0, 0.0); seeds_n];

    sim.set_assignments(assignments, sidelen);
    (seeds_n as u32, seeds, colors, sim)
}

fn init_colors(
    sidelen: u32,
    source: ImageBuffer<image::Rgb<u8>, Vec<u8>>,
) -> (Vec<SeedPos>, Vec<SeedColor>, usize) {
    let mut seeds = Vec::new();
    let mut colors = Vec::new();

    let width = source.width() as usize;
    let height = source.height() as usize;

    assert_eq!(width, height);

    let seeds_n = width * height;
    let pixelsize = sidelen as f32 / width as f32;

    for y in 0..width {
        for x in 0..width {
            let p = source.get_pixel(x as u32, y as u32);
            seeds.push(SeedPos {
                xy: [(x as f32 + 0.5) * pixelsize, (y as f32 + 0.5) * pixelsize],
            });
            colors.push(SeedColor {
                rgba: [
                    p[0] as f32 / 255.0,
                    p[1] as f32 / 255.0,
                    p[2] as f32 / 255.0,
                    1.0,
                ],
            });
        }
    }
    (seeds, colors, seeds_n)
}

#[derive(Clone, Copy)]
pub struct CellBody {
    srcx: f32,
    srcy: f32,
    dstx: f32,
    dsty: f32,

    velx: f32,
    vely: f32,

    accx: f32,
    accy: f32,

    dst_force: f32,
    age: u32,
    stroke_id: u32,
}

const PERSONAL_SPACE: f32 = 0.95;
const MAX_VELOCITY: f32 = 6.0;
const ALIGNMENT_FACTOR: f32 = 0.8;

fn factor_curve(x: f32) -> f32 {
    (x * x * x).min(1000.0)
}

impl CellBody {
    fn new(srcx: f32, srcy: f32, dstx: f32, dsty: f32, dst_force: f32) -> Self {
        Self {
            srcx,
            srcy,
            dstx,
            dsty,
            dst_force,
            velx: 0.0,
            vely: 0.0,
            accx: 0.0,
            accy: 0.0,
            age: 0,
            stroke_id: 0,
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_age(&mut self, age: u32) {
        self.age = age;
    }
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_dst_force(&mut self, force: f32) {
        self.dst_force = force;
    }
    #[cfg(not(target_arch = "wasm32"))]
    pub fn set_stroke_id(&mut self, stroke_id: u32) {
        self.stroke_id = stroke_id;
    }

    fn update(&mut self, pos: &mut SeedPos) {
        self.velx += self.accx;
        self.vely += self.accy;

        self.accx = 0.0;
        self.accy = 0.0;

        self.velx *= 0.97;
        self.vely *= 0.97;

        // self.velx = self.velx.clamp(-MAX_VELOCITY, MAX_VELOCITY);
        // self.vely = self.vely.clamp(-MAX_VELOCITY, MAX_VELOCITY);

        // pos.xy[0] += self.velx;
        // pos.xy[1] += self.vely;

        pos.xy[0] += self.velx.clamp(-MAX_VELOCITY, MAX_VELOCITY);
        pos.xy[1] += self.vely.clamp(-MAX_VELOCITY, MAX_VELOCITY);

        self.age += 1;
    }

    #[inline(always)]
    fn apply_dst_force(&mut self, pos: &SeedPos, sidelen: f32) {
        let elapsed = self.age as f32 / 60.0;
        let factor = if self.dst_force == 0.0 {
            0.1
        } else {
            factor_curve(elapsed * self.dst_force)
        };

        let dx = self.dstx - pos.xy[0];
        let dy = self.dsty - pos.xy[1];
        let dist = (dx * dx + dy * dy).sqrt();

        self.accx += (dx * dist * factor) / sidelen;
        self.accy += (dy * dist * factor) / sidelen;
    }

    /// Applies repulsive neighbor force. Takes raw floats to avoid SeedPos copies.
    /// Returns weight >= 0.
    #[inline(always)]
    fn apply_neighbour_force(
        &mut self,
        pos_x: f32,
        pos_y: f32,
        other_x: f32,
        other_y: f32,
        personal_space: f32,
    ) -> f32 {
        let dx = other_x - pos_x;
        let dy = other_y - pos_y;
        let dist = (dx * dx + dy * dy).sqrt();

        let weight = (1.0 / dist) * (personal_space - dist) / personal_space;

        if dist > 0.0 && dist < personal_space {
            self.accx -= dx * weight;
            self.accy -= dy * weight;
        } else if dist.abs() < f32::EPSILON {
            // if they are exactly on top of each other, push in a random direction
            let seed = (pos_x.to_bits() as u64) ^ ((pos_y.to_bits() as u64) << 32);
            let mut rng = frand::Rand::with_seed(seed);

            let r1 = rng.gen_range(0.0..1.0);
            let r2 = rng.gen_range(0.0..1.0);

            self.accx += (r1 - 0.5) * 0.1;
            self.accy += (r2 - 0.5) * 0.1;
        }

        weight.max(0.0)
    }

    #[inline(always)]
    fn apply_wall_force(&mut self, pos: &SeedPos, sidelen: f32, pixel_size: f32) {
        let personal_space = pixel_size * PERSONAL_SPACE * 0.5;

        if pos.xy[0] < personal_space {
            self.accx += (personal_space - pos.xy[0]) / personal_space;
        } else if pos.xy[0] > sidelen - personal_space {
            self.accx -= (pos.xy[0] - (sidelen - personal_space)) / personal_space;
        }

        if pos.xy[1] < personal_space {
            self.accy += (personal_space - pos.xy[1]) / personal_space;
        } else if pos.xy[1] > sidelen - personal_space {
            self.accy -= (pos.xy[1] - (sidelen - personal_space)) / personal_space;
        }
    }

    #[inline(always)]
    fn apply_stroke_attraction(&mut self, ix: f32, iy: f32, ox: f32, oy: f32, weight: f32) {
        self.accx += (ox - ix) * weight * 0.8;
        self.accy += (oy - iy) * weight * 0.8;
    }
}

pub struct Sim {
    //elapsed_frames: u32,
    pub cells: Vec<CellBody>,
    name: String,
    reversed: bool,
    // Pre-allocated grid buffers for neighbor search (avoids per-frame heap allocations)
    grid_counts: Vec<u32>,
    grid_offsets: Vec<u32>,
    grid_indices: Vec<u32>,
    particle_grid_cells: Vec<u32>,
}

impl Sim {
    pub fn new(name: String) -> Self {
        Self {
            cells: Vec::new(),
            //elapsed_frames: 0,
            name,
            reversed: false,
            grid_counts: Vec::new(),
            grid_offsets: Vec::new(),
            grid_indices: Vec::new(),
            particle_grid_cells: Vec::new(),
        }
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }

    // pub fn source_path(&self) -> PathBuf {
    //     self.source.clone()
    // }

    pub fn switch(&mut self) {
        for cell in &mut self.cells {
            mem::swap(&mut cell.srcx, &mut cell.dstx);
            mem::swap(&mut cell.srcy, &mut cell.dsty);
            cell.age = 0;
        }
        self.reversed = !self.reversed;
    }

    pub fn update(&mut self, positions: &mut [SeedPos], sidelen: u32) {
        let n = self.cells.len();
        let grid_size = (n as f32).sqrt();
        let grid_dim = grid_size as usize;
        let pixel_size = sidelen as f32 / grid_size;
        let grid_cells = grid_dim * grid_dim;
        let sidelen_f = sidelen as f32;
        let grid_max = grid_size - 1.0;
        let personal_space = pixel_size * PERSONAL_SPACE;

        // Precompute grid cell for each particle once (avoids 3x redundant division)
        self.particle_grid_cells.resize(n, 0);
        for (i, p) in positions.iter().enumerate() {
            let x = (p.xy[0] / pixel_size).floor().clamp(0.0, grid_max) as u32;
            let y = (p.xy[1] / pixel_size).floor().clamp(0.0, grid_max) as u32;
            self.particle_grid_cells[i] = y * grid_dim as u32 + x;
        }

        // Reuse pre-allocated flat grid buffers
        self.grid_counts.resize(grid_cells, 0);
        self.grid_counts.fill(0);

        // Count pass (using precomputed cells)
        for &cell_idx in &self.particle_grid_cells {
            self.grid_counts[cell_idx as usize] += 1;
        }

        // Prefix sum
        self.grid_offsets.resize(grid_cells + 1, 0);
        self.grid_offsets[0] = 0;
        for i in 0..grid_cells {
            self.grid_offsets[i + 1] = self.grid_offsets[i] + self.grid_counts[i];
        }

        // Placement pass (using precomputed cells)
        self.grid_indices.resize(n, 0);
        self.grid_counts.fill(0);
        for (i, &cell_idx) in self.particle_grid_cells.iter().enumerate() {
            let ci = cell_idx as usize;
            let write_idx = (self.grid_offsets[ci] + self.grid_counts[ci]) as usize;
            self.grid_indices[write_idx] = i as u32;
            self.grid_counts[ci] += 1;
        }

        // Apply wall + dst forces (independent per cell)
        for (i, cell) in self.cells.iter_mut().enumerate() {
            cell.apply_wall_force(&positions[i], sidelen_f, pixel_size);
            cell.apply_dst_force(&positions[i], sidelen_f);
        }

        // Neighbor interactions
        for i in 0..n {
            let pos_x = positions[i].xy[0];
            let pos_y = positions[i].xy[1];
            let col = (pos_x / pixel_size) as usize;
            let row = (pos_y / pixel_size) as usize;
            let my_stroke = self.cells[i].stroke_id;
            let mut avg_xvel = 0.0;
            let mut avg_yvel = 0.0;
            let mut count = 0.0;
            for dy in 0..=2 {
                for dx in 0..=2 {
                    if col + dx == 0
                        || row + dy == 0
                        || col + dx >= grid_dim
                        || row + dy >= grid_dim
                    {
                        continue;
                    }
                    let nindex = (row + dy - 1) * grid_dim + (col + dx - 1);

                    let start = self.grid_offsets[nindex] as usize;
                    let end = start + self.grid_counts[nindex] as usize;
                    for j in start..end {
                        let other = self.grid_indices[j] as usize;
                        if other == i {
                            continue;
                        }
                        let other_x = positions[other].xy[0];
                        let other_y = positions[other].xy[1];
                        let weight = self.cells[i].apply_neighbour_force(
                            pos_x,
                            pos_y,
                            other_x,
                            other_y,
                            personal_space,
                        );

                        if my_stroke == self.cells[other].stroke_id {
                            self.cells[i].apply_stroke_attraction(
                                pos_x, pos_y, other_x, other_y, weight,
                            );
                        }

                        avg_xvel += self.cells[other].velx * weight;
                        avg_yvel += self.cells[other].vely * weight;
                        count += weight;
                    }
                }
            }

            if count > 0.0 {
                avg_xvel /= count;
                avg_yvel /= count;

                self.cells[i].accx += (avg_xvel - self.cells[i].velx) * ALIGNMENT_FACTOR;
                self.cells[i].accy += (avg_yvel - self.cells[i].vely) * ALIGNMENT_FACTOR;
            }
        }

        for (index, cell) in self.cells.iter_mut().enumerate() {
            cell.update(&mut positions[index]);
        }
    }

    pub fn set_assignments(&mut self, assignments: Vec<usize>, sidelen: u32) {
        let width = (self.cells.len() as f32).sqrt();
        let pixelsize = sidelen as f32 / width;

        for (dst_idx, src_idx) in assignments.iter().enumerate() {
            let src_x = (src_idx % width as usize) as f32;
            let src_y = (src_idx / width as usize) as f32;
            let dst_x = (dst_idx % width as usize) as f32;
            let dst_y = (dst_idx / width as usize) as f32;
            let prev = self.cells[*src_idx];

            self.cells[*src_idx] = CellBody::new(
                (src_x + 0.5) * pixelsize,
                (src_y + 0.5) * pixelsize,
                (dst_x + 0.5) * pixelsize,
                (dst_y + 0.5) * pixelsize,
                prev.dst_force,
            );

            self.cells[*src_idx].age = prev.age;
            self.cells[*src_idx].stroke_id = prev.stroke_id;
        }
    }

    pub(crate) fn prepare_play(&mut self, positions: &mut [SeedPos], reverse: bool) {
        if self.reversed == reverse {
            for (i, cell) in self.cells.iter_mut().enumerate() {
                positions[i].xy[0] = cell.srcx;
                positions[i].xy[1] = cell.srcy;
                cell.age = 0;
            }
        } else {
            for (i, cell) in self.cells.iter().enumerate() {
                positions[i].xy[0] = cell.dstx;
                positions[i].xy[1] = cell.dsty;
            }
            self.switch();
        }
    }
}

// pub fn preset_path_to_name(source_dir: &Path) -> String {
//     source_dir
//         .file_stem()
//         .unwrap()
//         .to_string_lossy()
//         .into_owned()
// }

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::SeedPos;
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    fn make_test_sim(grid_size: usize, sidelen: u32) -> (Sim, Vec<SeedPos>) {
        let pixelsize = sidelen as f32 / grid_size as f32;
        let n = grid_size * grid_size;
        let mut seeds = Vec::with_capacity(n);
        let mut cells = Vec::with_capacity(n);
        for y in 0..grid_size {
            for x in 0..grid_size {
                seeds.push(SeedPos {
                    xy: [(x as f32 + 0.5) * pixelsize, (y as f32 + 0.5) * pixelsize],
                });
                cells.push(CellBody::new(
                    (x as f32 + 0.5) * pixelsize,
                    (y as f32 + 0.5) * pixelsize,
                    (x as f32 + 0.7) * pixelsize,
                    (y as f32 + 0.3) * pixelsize,
                    0.13,
                ));
            }
        }
        let mut sim = Sim::new("test".to_string());
        sim.cells = cells;
        (sim, seeds)
    }

    fn hash_state(seeds: &[SeedPos], cells: &[CellBody]) -> u64 {
        let mut hasher = DefaultHasher::new();
        for s in seeds {
            s.xy[0].to_bits().hash(&mut hasher);
            s.xy[1].to_bits().hash(&mut hasher);
        }
        for c in cells {
            c.velx.to_bits().hash(&mut hasher);
            c.vely.to_bits().hash(&mut hasher);
            c.accx.to_bits().hash(&mut hasher);
            c.accy.to_bits().hash(&mut hasher);
            c.age.hash(&mut hasher);
        }
        hasher.finish()
    }

    #[test]
    fn test_determinism() {
        let (mut s1, mut p1) = make_test_sim(16, 1024);
        let (mut s2, mut p2) = make_test_sim(16, 1024);
        for _ in 0..50 {
            s1.update(&mut p1, 1024);
            s2.update(&mut p2, 1024);
        }
        assert_eq!(
            hash_state(&p1, &s1.cells),
            hash_state(&p2, &s2.cells),
            "Simulation must be deterministic"
        );
    }

    #[test]
    fn test_golden_4x4() {
        let (mut sim, mut seeds) = make_test_sim(4, 256);
        for _ in 0..10 {
            sim.update(&mut seeds, 256);
        }
        let h = hash_state(&seeds, &sim.cells);
        eprintln!("GOLDEN_4X4 = {h}");
        assert_eq!(h, GOLDEN_4X4, "4x4 golden hash mismatch - behavior changed!");
    }

    #[test]
    fn test_golden_16x16() {
        let (mut sim, mut seeds) = make_test_sim(16, 1024);
        for _ in 0..20 {
            sim.update(&mut seeds, 1024);
        }
        let h = hash_state(&seeds, &sim.cells);
        eprintln!("GOLDEN_16X16 = {h}");
        assert_eq!(h, GOLDEN_16X16, "16x16 golden hash mismatch - behavior changed!");
    }

    #[test]
    fn test_golden_32x32() {
        let (mut sim, mut seeds) = make_test_sim(32, 2048);
        for _ in 0..30 {
            sim.update(&mut seeds, 2048);
        }
        let h = hash_state(&seeds, &sim.cells);
        eprintln!("GOLDEN_32X32 = {h}");
        assert_eq!(h, GOLDEN_32X32, "32x32 golden hash mismatch - behavior changed!");
    }

    // Golden values captured from original (pre-optimization) code
    const GOLDEN_4X4: u64 = 8875278218348756741;
    const GOLDEN_16X16: u64 = 9694425723178200883;
    const GOLDEN_32X32: u64 = 12208652155651837767;

    #[test]
    fn test_factor_curve() {
        assert_eq!(factor_curve(0.0), 0.0);
        assert_eq!(factor_curve(1.0), 1.0);
        assert_eq!(factor_curve(2.0), 8.0);
        assert_eq!(factor_curve(100.0), 1000.0);
    }

    #[test]
    fn test_wall_force_pushes_inward() {
        let pixel_size = 64.0;
        let sidelen = 256.0;

        let mut cell = CellBody::new(0.0, 128.0, 128.0, 128.0, 0.0);
        let near_left = SeedPos { xy: [5.0, 128.0] };
        cell.apply_wall_force(&near_left, sidelen, pixel_size);
        assert!(cell.accx > 0.0, "Should push right from left wall");

        let mut cell = CellBody::new(0.0, 128.0, 128.0, 128.0, 0.0);
        let near_right = SeedPos { xy: [251.0, 128.0] };
        cell.apply_wall_force(&near_right, sidelen, pixel_size);
        assert!(cell.accx < 0.0, "Should push left from right wall");

        let mut cell = CellBody::new(128.0, 0.0, 128.0, 128.0, 0.0);
        let near_top = SeedPos { xy: [128.0, 5.0] };
        cell.apply_wall_force(&near_top, sidelen, pixel_size);
        assert!(cell.accy > 0.0, "Should push down from top wall");

        let mut cell = CellBody::new(128.0, 0.0, 128.0, 128.0, 0.0);
        let near_bottom = SeedPos { xy: [128.0, 251.0] };
        cell.apply_wall_force(&near_bottom, sidelen, pixel_size);
        assert!(cell.accy < 0.0, "Should push up from bottom wall");
    }

    #[test]
    fn test_dst_force_direction() {
        let sidelen = 256.0;
        let mut cell = CellBody::new(0.0, 0.0, 200.0, 200.0, 0.13);
        cell.age = 60;
        let pos = SeedPos { xy: [100.0, 100.0] };
        cell.apply_dst_force(&pos, sidelen);
        assert!(cell.accx > 0.0, "Should pull toward dst (right)");
        assert!(cell.accy > 0.0, "Should pull toward dst (down)");
    }

    #[test]
    fn test_neighbour_repulsion() {
        let pixel_size = 64.0;
        let personal_space = pixel_size * PERSONAL_SPACE;
        let mut cell = CellBody::new(100.0, 100.0, 100.0, 100.0, 0.0);
        let weight = cell.apply_neighbour_force(100.0, 100.0, 110.0, 100.0, personal_space);
        assert!(weight > 0.0, "Close particles should have positive weight");
        assert!(cell.accx < 0.0, "Should repel away from neighbor");
    }

    #[test]
    fn test_neighbour_no_force_when_far() {
        let pixel_size = 64.0;
        let personal_space = pixel_size * PERSONAL_SPACE;
        let mut cell = CellBody::new(100.0, 100.0, 100.0, 100.0, 0.0);
        let weight = cell.apply_neighbour_force(100.0, 100.0, 200.0, 200.0, personal_space);
        assert_eq!(weight, 0.0, "Far particles should have zero weight");
        assert_eq!(cell.accx, 0.0, "No force when far apart");
        assert_eq!(cell.accy, 0.0, "No force when far apart");
    }

    #[test]
    fn test_switch() {
        let mut sim = Sim::new("test".to_string());
        sim.cells = vec![CellBody::new(10.0, 20.0, 30.0, 40.0, 0.1)];
        assert!(!sim.reversed);
        sim.switch();
        assert!(sim.reversed);
        assert_eq!(sim.cells[0].srcx, 30.0);
        assert_eq!(sim.cells[0].srcy, 40.0);
        assert_eq!(sim.cells[0].dstx, 10.0);
        assert_eq!(sim.cells[0].dsty, 20.0);
        assert_eq!(sim.cells[0].age, 0);
    }

    #[test]
    fn test_update_preserves_count() {
        let (mut sim, mut seeds) = make_test_sim(8, 512);
        let n = seeds.len();
        for _ in 0..20 {
            sim.update(&mut seeds, 512);
        }
        assert_eq!(seeds.len(), n, "Seed count must not change");
        assert_eq!(sim.cells.len(), n, "Cell count must not change");
    }

    #[test]
    fn test_positions_converge_to_dst() {
        let (mut sim, mut seeds) = make_test_sim(4, 256);
        // Run for many steps
        for _ in 0..500 {
            sim.update(&mut seeds, 256);
        }
        // After many steps, positions should be close to destinations
        for (i, cell) in sim.cells.iter().enumerate() {
            let dx = (seeds[i].xy[0] - cell.dstx).abs();
            let dy = (seeds[i].xy[1] - cell.dsty).abs();
            assert!(
                dx < 20.0 && dy < 20.0,
                "Seed {} should converge near dst: pos=({:.1},{:.1}) dst=({:.1},{:.1})",
                i, seeds[i].xy[0], seeds[i].xy[1], cell.dstx, cell.dsty
            );
        }
    }

    #[test]
    fn bench_update_128x128() {
        let (mut sim, mut seeds) = make_test_sim(128, 2048);
        let iters = 50;
        // Warm up
        for _ in 0..5 {
            sim.update(&mut seeds, 2048);
        }
        let start = std::time::Instant::now();
        for _ in 0..iters {
            sim.update(&mut seeds, 2048);
        }
        let elapsed = start.elapsed();
        let avg_ms = elapsed.as_secs_f64() * 1000.0 / iters as f64;
        eprintln!(
            "PERF bench_update_128x128: {avg_ms:.3}ms avg ({iters} iters, {:.1}ms total)",
            elapsed.as_secs_f64() * 1000.0,
        );
        // For 60fps need <16ms per frame
        assert!(
            avg_ms < 100.0,
            "Sim::update() too slow: {avg_ms:.1}ms (target <16ms)"
        );
    }

    #[test]
    fn bench_update_64x64() {
        let (mut sim, mut seeds) = make_test_sim(64, 1024);
        let iters = 100;
        for _ in 0..5 {
            sim.update(&mut seeds, 1024);
        }
        let start = std::time::Instant::now();
        for _ in 0..iters {
            sim.update(&mut seeds, 1024);
        }
        let elapsed = start.elapsed();
        let avg_ms = elapsed.as_secs_f64() * 1000.0 / iters as f64;
        eprintln!(
            "PERF bench_update_64x64: {avg_ms:.3}ms avg ({iters} iters, {:.1}ms total)",
            elapsed.as_secs_f64() * 1000.0,
        );
    }

    #[test]
    fn bench_update_256x256() {
        let (mut sim, mut seeds) = make_test_sim(256, 4096);
        let iters = 10;
        for _ in 0..2 {
            sim.update(&mut seeds, 4096);
        }
        let start = std::time::Instant::now();
        for _ in 0..iters {
            sim.update(&mut seeds, 4096);
        }
        let elapsed = start.elapsed();
        let avg_ms = elapsed.as_secs_f64() * 1000.0 / iters as f64;
        eprintln!(
            "PERF bench_update_256x256: {avg_ms:.3}ms avg ({iters} iters, {:.1}ms total)",
            elapsed.as_secs_f64() * 1000.0,
        );
        // 256x256 is a very heavy workload; check it's still reasonable
        assert!(
            avg_ms < 500.0,
            "256x256 update too slow: {avg_ms:.1}ms"
        );
    }
}
