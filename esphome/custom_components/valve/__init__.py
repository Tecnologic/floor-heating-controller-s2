import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

valve_ns = cg.esphome_ns.namespace('valve')
Valve = valve_ns.class_('Valve', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Valve)
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
