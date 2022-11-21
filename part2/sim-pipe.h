#include "machine.h"


/*define buffer between fetch and decode stage*/
struct ifid_buf {
  md_inst_t inst;	    /* instruction that has been fetched */
  md_addr_t pc;	        /* pc value of current instruction */
};


/*define buffer between decode and execute stage*/
struct idex_buf {
  md_inst_t inst;		/* instruction in ID stage */ 
  md_addr_t pc;
  int opcode;			/* operation number */
  byte_t rd;
  byte_t rs;
  byte_t rt;
  sword_t rs_val;  // for signed arithmetics
  sword_t rt_val;
};

/*define buffer between execute and memory stage*/
struct exmem_buf{
  md_inst_t inst;		/* instruction in EX stage */
  md_addr_t pc;
  bool_t load_mem;
  bool_t write_rt;
  bool_t write_rd;
  bool_t write_enable;
  word_t ALUout;
  word_t data_in;
  byte_t rd;
  byte_t rt;
  bool_t go_branch;
  word_t branch_addr;
};

/*define buffer between memory and writeback stage*/
struct memwb_buf{
  md_inst_t inst;		/* instruction in MEM stage */
  md_addr_t pc;
  bool_t write_rt;
  bool_t write_rd;
  bool_t load_mem;
  word_t from_mem;
  word_t ALUout;
  byte_t rt;
  byte_t rd;
};

struct wb_buf{
  md_inst_t inst;
  md_addr_t pc;
};
  

/*do fetch stage*/
void do_if();

/*do decode stage*/
void do_id();

/*do execute stage*/
void do_ex();

/*do memory stage*/
void do_mem();

/*do write_back to register*/
void do_wb();

void print_trace();


#define MD_FETCH_INSTI(INST, MEM, PC)					\
  { INST.a = MEM_READ_WORD(mem, (PC));					\
    INST.b = MEM_READ_WORD(mem, (PC) + sizeof(word_t)); }

#define SET_OPCODE(OP, INST) ((OP) = ((INST).a & 0xff)) 

#define RSI(INST)		(INST.b >> 24& 0xff)		/* reg source #1 */
#define RTI(INST)		((INST.b >> 16) & 0xff)		/* reg source #2 */
#define RDI(INST)		((INST.b >> 8) & 0xff)		/* reg dest */

#define IMMI(INST)	((int)((/* signed */short)(INST.b & 0xffff)))	/*get immediate value*/
#define TARGI(INST)	(INST.b & 0x3ffffff)		/*jump target*/
