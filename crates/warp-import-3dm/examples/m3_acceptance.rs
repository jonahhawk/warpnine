//! Reproducible Rhino -> selected CAM -> simulation -> every post -> reopen acceptance.
use glam::DVec3;
use warp_cam::{ops::*, post::*, *};
use warp_core::{interchange::*, scene::*, selection::*};
fn main() -> anyhow::Result<()> {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let out = root.join("target/m3-acceptance");
    std::fs::create_dir_all(&out)?;
    let imported = warp_import_3dm::OpenNurbsImporter.read(
        &root.join("fixtures/rhino-m2/parts-mm-v8.3dm"),
        &ImportOptions::default(),
    )?;
    let objects = imported.scene.resolved()?;
    let perimeter = objects
        .iter()
        .find(|o| {
            o.name == "Block perimeter"
                && o.reference
                    .instances
                    .first()
                    .is_some_and(|i| i.0.contains("00000015-"))
        })
        .unwrap();
    let hole = objects
        .iter()
        .find(|o| o.name == "Block hole" && o.reference.instances == perimeter.reference.instances)
        .unwrap();
    let follow = objects
        .iter()
        .find(|o| o.name == "Mixed polycurve")
        .unwrap();
    let mut tools = warp_tools::ToolLibrary::starter();
    let mut cutter = warp_tools::Tool::new(
        20,
        "M3 fixture 0.5mm flat",
        warp_tools::ToolShape::FlatEndMill,
        0.5,
    );
    cutter.flute_length = 3.;
    cutter.stickout = 8.;
    cutter.shank_diameter = 3.;
    tools.tools.push(cutter);
    let mut fixture_tools = tools.clone();
    fixture_tools.tools.retain(|t| t.number == 20);
    fixture_tools.holders.clear();
    fixture_tools.save(&out.join("tools.json"))?;
    let mut project = Project::default();
    project.name = "M3 Rhino acceptance".into();
    project.post = PostConfig::mach3().name;
    project.stock = Stock {
        min: DVec3::new(-2., -2., -2.),
        max: DVec3::new(45., 38., 0.),
    };
    project.ensure_document()?;
    project.scene = Some(imported.scene);
    let select = |refs: Vec<GeometryRef>| Selection {
        mode: SelectionMode::Selected,
        references: refs,
    };
    let cases = vec![
        (
            "Island pocket",
            OpKind::Pocket {
                pattern: PocketPattern::Offset,
                stepover: 0.2,
                finish_pass: true,
                inside_out: false,
                order: CutOrder::LayerFirst,
            },
            vec![perimeter.reference.clone(), hole.reference.clone()],
        ),
        (
            "Profile Part B",
            OpKind::Profile {
                side: Side::Outside,
                finish_passes: 0,
                finish_stepover: 0.,
                spring_pass: false,
                lead_radius: 0.,
                tabs: None,
            },
            vec![perimeter.reference.clone()],
        ),
        (
            "Hole pocket",
            OpKind::HolePocket {
                stepover: 0.2,
                pitch: 0.3,
            },
            vec![hole.reference.clone()],
        ),
        (
            "Follow loose curve",
            OpKind::Engrave,
            vec![follow.reference.clone()],
        ),
    ];
    for (i, (name, kind, refs)) in cases.into_iter().enumerate() {
        let mut common = Common::for_tool(tools.tool(20).unwrap());
        common.id = OperationId(format!("m3/{i}"));
        common.setup = project.active_setup.clone();
        common.heights.bottom = -0.5;
        common.stepdown = 0.25;
        common.entry = Entry::Ramp { angle: 3. };
        let mut geometry = ControlGeometry::default();
        *geometry.get_mut(warp_cam::document::primary_role(&kind)) = select(refs);
        common.geometry = Some(geometry);
        project.operations.push(Operation {
            name: name.into(),
            enabled: true,
            common,
            kind,
        });
    }
    let fixture = project.scene.as_mut().unwrap().add(
        "M3 clamp bounding box".into(),
        Geometry::Mesh(warp_core::Mesh {
            tris: vec![[
                DVec3::new(42., 32., 0.),
                DVec3::new(44., 32., 0.),
                DVec3::new(44., 34., 4.),
            ]],
            provenance: None,
        }),
    );
    let layer = project.scene.as_ref().unwrap().layers[0].id.clone();
    project
        .scene
        .as_mut()
        .unwrap()
        .objects
        .iter_mut()
        .find(|o| o.id == fixture)
        .unwrap()
        .layer = layer;
    project.setups[0].fixtures = select(vec![GeometryRef::object(fixture)]);
    project.refresh_view()?;
    let paths: Vec<_> = project
        .operations
        .iter()
        .map(|o| generate(o, &project, &tools))
        .collect::<anyhow::Result<_>>()?;
    project.save(&out.join("rhino-job.warp"))?;
    let restored = Project::load(&out.join("rhino-job.warp"))?;
    for (op, path) in restored.operations.iter().zip(&paths) {
        assert_eq!(generate(op, &restored, &tools)?.moves, path.moves);
    }
    let mut stock = warp_sim::Stock::new(project.stock.min, project.stock.max, 512)
        .map_err(anyhow::Error::msg)?;
    let mut segments = Vec::new();
    let mut start = DVec3::new(0., 0., 15.);
    for path in &paths {
        for (source, (from, to, rapid, feed)) in path.segments(start, 0.005).into_iter().enumerate()
        {
            segments.push(warp_sim::SimSegment {
                from,
                to,
                rapid,
                feed,
                tool: path.tool,
                source,
            });
            start = to;
        }
    }
    let mut player = warp_sim::Player::new(segments);
    player.run_to_end(&mut stock, &|n| tools.tool(n).cloned());
    anyhow::ensure!(
        player.collisions.is_empty(),
        "Rapid stock collisions: {}",
        player.collisions.len()
    );
    let mut verified = Vec::new();
    for (i, cfg) in PostConfig::presets().iter().enumerate() {
        let code = post_checked(cfg, &project.name, &paths, &tools)?;
        verified.push(format!(
            "{}: {} motion/dwell events verified",
            cfg.name,
            warp_cam::verification::verify_post(cfg, &paths, &code)?
        ));
        let program = warp_gcode::Interpreter::default().run_str(&code)?;
        anyhow::ensure!(!program.motions.is_empty(), "Empty output");
        std::fs::write(out.join(format!("post-{i}.{}", cfg.file_extension)), code)?;
    }
    let report=format!("4 Rhino-selected operations generated and identical after reopen.\n5 posts interpreted and checked against the source toolpaths.\n{}\n512-cell stock simulation: {} rapid collisions; {:.3} mm3 removed.\n",verified.join("\n"),player.collisions.len(),stock.removed_volume());
    std::fs::write(out.join("report.txt"), &report)?;
    // A second interactive fixture exercises directed open walls with the same stock/tool.
    let mut open_job = project.clone();
    open_job.name = "M3 open wall acceptance".into();
    let wall = warp_core::Polyline2::new(
        vec![
            glam::DVec2::new(-2., 10.),
            glam::DVec2::new(20., 10.),
            glam::DVec2::new(20., 25.),
            glam::DVec2::new(-2., 25.),
        ],
        false,
    );
    let id = open_job.scene.as_mut().unwrap().add(
        "Open pocket wall — reverse to change side".into(),
        Geometry::polyline(&wall),
    );
    let layer = open_job.scene.as_ref().unwrap().layers[0].id.clone();
    open_job
        .scene
        .as_mut()
        .unwrap()
        .objects
        .iter_mut()
        .find(|o| o.id == id)
        .unwrap()
        .layer = layer;
    let mut operation = open_job.operations[0].clone();
    operation.name = "Open wall pocket".into();
    operation.kind = OpKind::OpenPocket { stepover: 0.2 };
    operation.common.geometry.as_mut().unwrap().regions = select(vec![GeometryRef::object(id)]);
    open_job.operations = vec![operation];
    open_job.refresh_view()?;
    let open_path = generate(&open_job.operations[0], &open_job, &tools)?;
    std::fs::write(
        out.join("open-wall.tap"),
        post_checked(&PostConfig::mach3(), &open_job.name, &[open_path], &tools)?,
    )?;
    open_job.save(&out.join("open-wall-job.warp"))?;
    print!("{report}");
    Ok(())
}
