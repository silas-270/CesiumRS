use crate::globe::quadtree::TileId;
use crate::globe::geometry::TileMesh;
use async_trait::async_trait;

/// Base trait for providing imagery (textures) for tiles.
#[async_trait]
pub trait ImageryProvider: Send + Sync {
    /// Requests the raw image bytes for a specific tile.
    async fn request_image(&self, id: &TileId) -> Result<Vec<u8>, String>;
    
    /// The minimum zoom level this provider supports.
    fn minimum_level(&self) -> u32 { 0 }
    
    /// The maximum zoom level this provider supports.
    fn maximum_level(&self) -> u32 { 18 }
}

/// Base trait for providing geometry (terrain) for tiles.
#[async_trait]
pub trait TerrainProvider: Send + Sync {
    /// Requests the geometry for a specific tile.
    async fn request_tile_geometry(&self, id: &TileId) -> Result<TileMesh, String>;
    
    /// The minimum zoom level this provider supports.
    fn minimum_level(&self) -> u32 { 0 }
    
    /// The maximum zoom level this provider supports.
    fn maximum_level(&self) -> u32 { 14 }
    
    /// Whether this provider should request vertex normals from the server.
    fn request_vertex_normals(&self) -> bool { false }
    
    /// Whether this provider should request a water mask from the server.
    fn request_water_mask(&self) -> bool { false }
}

// ============================================================================
// Concrete Implementations
// ============================================================================

/// Provides default OpenStreetMap imagery.
pub struct OpenStreetMapImageryProvider {
    client: reqwest::Client,
}

impl OpenStreetMapImageryProvider {
    pub fn new(client: reqwest::Client) -> Self {
        Self { client }
    }
}

#[async_trait]
impl ImageryProvider for OpenStreetMapImageryProvider {
    async fn request_image(&self, id: &TileId) -> Result<Vec<u8>, String> {
        let url = format!("https://tile.openstreetmap.org/{}/{}/{}.png", id.z, id.x, id.y);
        
        let req = self.client.get(&url)
            .header("User-Agent", "CesiumRS/0.1.0")
            .build()
            .map_err(|e| e.to_string())?;

        let response = self.client.execute(req).await.map_err(|e| e.to_string())?;
        
        if response.status().is_success() {
            let bytes = response.bytes().await.map_err(|e| e.to_string())?;
            Ok(bytes.to_vec())
        } else {
            Err(format!("HTTP {}", response.status()))
        }
    }
}

/// Provides a perfectly smooth WGS84 ellipsoid (no actual terrain downloaded).
pub struct EllipsoidTerrainProvider;

impl EllipsoidTerrainProvider {
    pub fn new() -> Self {
        Self
    }
}

#[async_trait]
impl TerrainProvider for EllipsoidTerrainProvider {
    async fn request_tile_geometry(&self, id: &TileId) -> Result<TileMesh, String> {
        // We use tokio::task::spawn_blocking because generate_ellipsoid is a CPU-bound math task.
        let id_copy = *id;
        let mesh = tokio::task::spawn_blocking(move || {
            TileMesh::generate(&id_copy, 16)
        }).await.map_err(|e| e.to_string())?;
        
        Ok(mesh)
    }
}

/// Provides 3D terrain by fetching Quantized-Mesh (.terrain) files.
pub struct CesiumTerrainProvider {
    client: reqwest::Client,
    base_url: String,
}

impl CesiumTerrainProvider {
    pub fn new(client: reqwest::Client, base_url: String) -> Self {
        Self { client, base_url }
    }
}

#[async_trait]
impl TerrainProvider for CesiumTerrainProvider {
    async fn request_tile_geometry(&self, id: &TileId) -> Result<TileMesh, String> {
        let url = format!("{}/{}/{}/{}.terrain", self.base_url, id.z, id.x, id.y);
        
        let req = self.client.get(&url)
            .header("User-Agent", "CesiumRS/0.1.0")
            // Cesium's server often requires an Accept header for quantized-mesh
            .header("Accept", "application/vnd.quantized-mesh,application/octet-stream;q=0.9")
            .build()
            .map_err(|e| e.to_string())?;

        let response = self.client.execute(req).await.map_err(|e| e.to_string())?;
        
        if !response.status().is_success() {
            return Err(format!("HTTP {}", response.status()));
        }

        let bytes = response.bytes().await.map_err(|e| e.to_string())?.to_vec();

        let id_copy = *id;
        let mesh = tokio::task::spawn_blocking(move || {
            let qm = crate::globe::terrain_parser::parse_quantized_mesh(&bytes)
                .map_err(|e| format!("Parse error: {:?}", e))?;
            
            Ok::<TileMesh, String>(TileMesh::from_quantized_mesh(&qm, &id_copy))
        }).await.map_err(|e| e.to_string())??;
        
        Ok(mesh)
    }

    fn request_vertex_normals(&self) -> bool {
        true
    }
    
    fn request_water_mask(&self) -> bool {
        false
    }
}
