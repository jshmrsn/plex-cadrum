/// Stable failure classification independent of a specific OCCT algorithm.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FailureCategory {
	Cancelled,
	InvalidInput,
	NoSolution,
	InvalidResult,
	ResourceLimit,
	BridgeDefect,
	AlgorithmFailed,
	Io,
}

/// Structured details captured when an OCCT exception crosses the C++ bridge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OperationFailure {
	pub operation: String,
	pub stage: String,
	pub exception_type: Option<String>,
	pub message: String,
	pub category: FailureCategory,
	pub status: Option<i32>,
}

/// Errors that can occur during CAD operations.
#[derive(Debug)]
pub enum Error {
	/// OCCT threw a caught native exception with structured diagnostics.
	OperationFailed(OperationFailure),

	/// A public modeling entry point rejected malformed, non-finite, or
	/// geometrically degenerate input before entering OCCT.
	InvalidInput(String),

	/// STEP file read failed (invalid format or corrupted data).
	StepReadFailed,

	/// BRep file read failed (invalid format or corrupted data).
	BrepReadFailed,

	/// STEP file write failed.
	StepWriteFailed,

	/// BRep file write failed.
	BrepWriteFailed,

	/// Triangulation/meshing failed.
	TriangulationFailed,

	/// An exact point projection failed to converge or the topology has no
	/// usable underlying geometry.
	ProjectionFailed(&'static str),

	/// A batched topology query returned inconsistent data.
	TopologyQueryFailed,

	/// The caller cooperatively cancelled an OCCT operation.
	Cancelled,

	/// Boolean operation (fuse/cut/common) failed.
	BooleanOperationFailed,

	/// 単一 Solid を期待する演算 (`+`/`-`/`*` 演算子) で結果の Solid 数が 1 でなかった。
	/// `usize` は実際の結果 Solid 数 (0 = 非交差/全削除、2+ = 結果が複数ピースに分割)。
	/// 戻り値が複数ピースになりうる場合は `Solid::boolean_union/subtract/intersect` を
	/// 直接使い `Vec<Solid>` を受け取ること。
	OneFailed(usize),

	/// Shape cleaning (UnifySameDomain) failed.
	CleanFailed,

	/// Helix edge construction failed (e.g. degenerate parameters).
	HelixFailed,

	/// Extrusion (`Solid::extrude`) failed: empty profile, zero-length
	/// direction, or profile not closed.
	ExtrudeFailed,

	/// Pipe sweep (`Solid::sweep`) failed: profile not closed, edges not
	/// connectable into a wire, or `BRepOffsetAPI_MakePipe` returned no shape.
	SweepFailed,

	/// Shell / hollow (`Solid::shell` via `BRepOffsetAPI_MakeThickSolid`)
	/// failed: thickness sign incompatible with geometry, sharp corners
	/// yielding a self-intersecting offset surface, or OCCT internal failure.
	ShellFailed,

	/// Fillet (`Solid::fillet_edges` via `BRepFilletAPI_MakeFillet`) failed:
	/// radius too large for the local geometry, tangent discontinuity along
	/// the selected edge chain, or an edge not belonging to `self` was passed.
	FilletFailed,

	/// Chamfer (`Solid::chamfer_edges` via `BRepFilletAPI_MakeChamfer`) failed:
	/// distance too large for the local geometry, tangent discontinuity along
	/// the selected edge chain, or an edge not belonging to `self` was passed.
	ChamferFailed,

	/// Lofting (`Solid::loft` / `BRepOffsetAPI_ThruSections`) failed: section
	/// count too low, section wire ill-formed, or OCCT internal failure.
	/// The string identifies which precondition or stage failed.
	LoftFailed(String),

	/// Sewing (`Solid::sew` / `BRepBuilderAPI_Sewing`) failed: the faces do
	/// not form exactly one closed shell within the given tolerance (gaps,
	/// overlaps, multiple disconnected shells, or stray faces). The string
	/// identifies which precondition or stage failed.
	SewFailed(String),

	/// Surface offset (`Solid::offset_surface` / `BRepOffsetAPI_MakeOffsetShape`)
	/// failed: the offset surfaces self-intersect (thin walls/slots thinner
	/// than 2x the offset magnitude) or OCCT rejected the join. The string
	/// carries the offending parameters.
	OffsetFailed(String),

	/// B-spline solid (`Solid::bspline`) construction failed: grid too small,
	/// surface interpolation rejected the input, or sewing/capping failed.
	/// The string identifies which stage failed and with what parameters.
	BsplineFailed(String),

	/// Edge construction failed due to degenerate input (e.g. collinear arc
	/// points, zero-length line, negative radius). The string describes which
	/// constructor failed and with which parameters.
	InvalidEdge(String),

	/// SVG export (HLR projection) failed.
	SvgExportFailed,

	/// PNG export failed (rasterizer / encoder / writer).
	PngExportFailed,

	/// STL write failed.
	StlWriteFailed,

	/// glTF (GLB binary) write failed.
	GltfWriteFailed,

	/// Invalid color string (unrecognized name or invalid hex format).
	InvalidColor(String),

	/// Unknown error.
	Unknown(String),
}

impl std::fmt::Display for Error {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			Error::OperationFailed(failure) => {
				write!(f, "{} failed during {}", failure.operation, failure.stage)?;
				if let Some(exception_type) = &failure.exception_type {
					write!(f, " ({exception_type})")?;
				}
				if !failure.message.is_empty() {
					write!(f, ": {}", failure.message)?;
				}
				Ok(())
			}
			Error::InvalidInput(message) => write!(f, "Invalid modeling input: {message}"),
			Error::StepReadFailed => write!(f, "STEP read failed"),
			Error::BrepReadFailed => write!(f, "BRep read failed"),
			Error::StepWriteFailed => write!(f, "STEP write failed"),
			Error::BrepWriteFailed => write!(f, "BRep write failed"),
			Error::TriangulationFailed => write!(f, "Triangulation failed"),
			Error::ProjectionFailed(entity) => write!(f, "{entity} projection failed"),
			Error::TopologyQueryFailed => write!(f, "Topology query failed"),
			Error::Cancelled => write!(f, "Operation cancelled"),
			Error::BooleanOperationFailed => write!(f, "Boolean operation failed"),
			Error::OneFailed(n) => write!(f, "Expected exactly one resulting Solid, got {}", n),
			Error::CleanFailed => write!(f, "Shape clean failed"),
			Error::HelixFailed => write!(f, "Helix failed"),
			Error::ExtrudeFailed => write!(f, "Extrude failed"),
			Error::SweepFailed => write!(f, "Sweep failed"),
			Error::ShellFailed => write!(f, "Shell failed"),
			Error::FilletFailed => write!(f, "Fillet failed"),
			Error::ChamferFailed => write!(f, "Chamfer failed"),
			Error::LoftFailed(msg) => write!(f, "Loft failed: {}", msg),
			Error::SewFailed(msg) => write!(f, "Sew failed: {}", msg),
			Error::OffsetFailed(msg) => write!(f, "Offset failed: {}", msg),
			Error::BsplineFailed(msg) => write!(f, "Bspline failed: {}", msg),
			Error::InvalidEdge(msg) => write!(f, "Invalid edge: {}", msg),
			Error::SvgExportFailed => write!(f, "SVG export failed"),
			Error::PngExportFailed => write!(f, "PNG export failed"),
			Error::StlWriteFailed => write!(f, "STL write failed"),
			Error::GltfWriteFailed => write!(f, "glTF write failed"),
			Error::InvalidColor(s) => write!(f, "Invalid color: \"{}\"", s),
			Error::Unknown(msg) => write!(f, "Unknown error: {}", msg),
		}
	}
}

impl Error {
	/// Return a stable category suitable for adapter policy and UI recovery.
	pub fn category(&self) -> FailureCategory {
		match self {
			Self::OperationFailed(failure) => failure.category,
			Self::Cancelled => FailureCategory::Cancelled,
			Self::InvalidInput(_) | Self::InvalidEdge(_) | Self::InvalidColor(_) => FailureCategory::InvalidInput,
			Self::ProjectionFailed(_) => FailureCategory::NoSolution,
			Self::TopologyQueryFailed | Self::TriangulationFailed | Self::OneFailed(_) => FailureCategory::InvalidResult,
			Self::StepReadFailed | Self::BrepReadFailed | Self::StepWriteFailed | Self::BrepWriteFailed | Self::SvgExportFailed | Self::PngExportFailed | Self::StlWriteFailed | Self::GltfWriteFailed => FailureCategory::Io,
			Self::Unknown(_) => FailureCategory::BridgeDefect,
			Self::BooleanOperationFailed | Self::CleanFailed | Self::HelixFailed | Self::ExtrudeFailed | Self::SweepFailed | Self::ShellFailed | Self::FilletFailed | Self::ChamferFailed | Self::LoftFailed(_) | Self::SewFailed(_) | Self::OffsetFailed(_) | Self::BsplineFailed(_) => FailureCategory::AlgorithmFailed,
		}
	}

	/// Return the stable native/adapter stage associated with this failure.
	pub fn stage(&self) -> &str {
		match self {
			Self::OperationFailed(failure) => &failure.stage,
			Self::Cancelled => "cancelled",
			Self::ProjectionFailed(_) => "project",
			Self::TopologyQueryFailed => "topology_snapshot",
			Self::TriangulationFailed => "mesh",
			Self::StepReadFailed | Self::BrepReadFailed => "read",
			Self::StepWriteFailed | Self::BrepWriteFailed => "write",
			Self::InvalidInput(_) | Self::InvalidEdge(_) | Self::InvalidColor(_) => "validate_input",
			Self::SvgExportFailed | Self::PngExportFailed | Self::StlWriteFailed | Self::GltfWriteFailed => "export",
			Self::OneFailed(_) => "validate_result",
			_ => "occt_build",
		}
	}

	/// Whether a caller may safely keep displaying its preceding coherent result.
	pub fn may_keep_last_valid_result(&self) -> bool {
		!matches!(self.category(), FailureCategory::BridgeDefect | FailureCategory::ResourceLimit)
	}
}

impl std::error::Error for Error {}
