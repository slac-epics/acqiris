TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += acqiris_board.db
DB += acqiris_board.substitutions
DB += acqiris_channel.template
DB += acqiris_channel_extras.template
DB += acqiris_module.template
DB += acqiris_module_extras.template
DB += acqiris_trigger.template

# Create and install archive files
ARCHIVE += acqiris_channel.tpl-arch
ARCHIVE += acqiris_module.tpl-arch
ARCHIVE += acqiris_trigger.tpl-arch
ARCHIVE += acqiris_board.sub-arch
ARCHIVE += acqiris_board.archive

# Create and install autosave files
REQ += acqiris_channel.tpl-req
REQ += acqiris_module.tpl-req
REQ += acqiris_trigger.tpl-req
REQ += acqiris_board.sub-req
REQ += acqiris_board.req

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


