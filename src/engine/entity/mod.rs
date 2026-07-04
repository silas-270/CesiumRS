use glam::{DVec3, DQuat};
use crate::engine::property::Property;

pub struct Entity {
    pub id: String,
    pub position: Option<Box<dyn Property<DVec3>>>,
    pub orientation: Option<Box<dyn Property<DQuat>>>,
}

impl Entity {
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            position: None,
            orientation: None,
        }
    }
}

pub struct EntityCollection {
    entities: Vec<Entity>,
}

impl EntityCollection {
    pub fn new() -> Self {
        Self {
            entities: Vec::new(),
        }
    }

    pub fn add(&mut self, entity: Entity) {
        self.entities.push(entity);
    }

    pub fn get(&self, id: &str) -> Option<&Entity> {
        self.entities.iter().find(|e| e.id == id)
    }

    pub fn get_mut(&mut self, id: &str) -> Option<&mut Entity> {
        self.entities.iter_mut().find(|e| e.id == id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &Entity> {
        self.entities.iter()
    }
}
