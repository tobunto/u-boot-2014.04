#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/errno.h>

#include <asm/arch/x1000.h>

void pll_init(void)
{
	cpm_writel(0x55700000,CPM_CPCCR);
	while((cpm_readl(CPM_CPCSR) & 0xf0000007) != 0xf0000000);

	cpm_writel(0xa9000120, CPM_CPAPCR);
	while(!(cpm_readl(CPM_CPAPCR) & (1 << 10)));

	cpm_writel(0x98000080, CPM_CPMPCR);
	while(!(cpm_readl(CPM_CPMPCR) & 1));

	cpm_writel(0x55752210, CPM_CPCCR);
	while(cpm_readl(CPM_CPCSR) & 7);

	cpm_writel(0x9a052210, CPM_CPCCR);
	while(!(cpm_readl(CPM_CPCSR) & 0xf0000000));
}
